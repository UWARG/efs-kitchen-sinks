#include "ftl.h"



extern HAL_StatusTypeDef erase_4k(uint32_t addr24);
extern void              read_data(uint32_t addr24, uint8_t *out, uint16_t len);


/*
 * Internal FTL state (file-local).
 *
 * We maintain a minimal view of the circular log:
 *
 *   g_head_idx : index (0..FTL_NUM_UNITS-1) of newest VALID record,
 *                or 0xFFFFFFFF if none exist.
 *
 *   g_tail_idx : index of oldest VALID record,
 *                or 0xFFFFFFFF if none exist.
 *
 *   g_next_idx : index where the next append() will write.
 *
 *   g_next_id  : record ID that will be assigned to the next append().
 *
 *   g_mounted  : set to true after a successful ftl_mount().
 *
 * For now these are only used inside this file; we can add public
 * accessors later once we implement append/read APIs.
 */

static uint32_t g_head_idx  = 0xFFFFFFFFu;
static uint32_t g_tail_idx  = 0xFFFFFFFFu;
static uint32_t g_next_idx  = 0u;
static uint32_t g_next_id   = 0u;
static bool     g_mounted   = false;



// Helper function to compute the base address in flash for a given unit index
static inline uint32_t ftl_unit_base_addr(uint32_t unit_index)
{
    return (unit_index * FTL_UNIT_SIZE);  // FTL_DATA_BASE is 0, so this is fine
}



/*
 * ftl_format
 *
 * Erase every 4 KB unit in the FTL region.
 * Since we use the entire external flash as the log, this is:
 *
 *   for each unit i in [0 .. FTL_NUM_UNITS-1]:
 *       erase_4k(i * 4KB)
 *
 * After this, every header is effectively:
 *   status = 0xFFFF (FTL_STATUS_EMPTY)
 *   id     = 0xFFFFFFFF (unused)
 *   length = 0xFFFF (ignored)
 *   crc    = 0xFFFFFFFF (ignored)
 */
int ftl_format(void)
{
	HAL_StatusTypeDef st = erase_full();
	if (st != HAL_OK) {
		return -1;
	}

    // Reset in-RAM state
    g_head_idx = 0xFFFFFFFFu;
    g_tail_idx = 0xFFFFFFFFu;
    g_next_idx = 0u;    // next write will go to unit 0
    g_next_id  = 0u;    // first record will have ID 0
    g_mounted  = false;

    return 0;
}


/*
 * ftl_mount
 *
 * Scan all units, looking at the header in the first 256 B page of each.
 *
 * For each unit i:
 *   - Read ftl_record_header_t from address (i * FTL_UNIT_SIZE).
 *   - If status == FTL_STATUS_EMPTY && id == 0xFFFFFFFF:
 *         -> block is completely unused, skip.
 *   - Else if status == FTL_STATUS_VALID and length is sane:
 *         -> treat as a candidate record, and use hdr.id to track:
 *              * min_id: oldest (tail)
 *              * max_id: newest (head)
 *
 * For now we DO NOT recompute CRC here; we simply trust VALID headers.
 * CRC checking can be added later.
 *
 * After scanning all units:
 *   - If no VALID records were found:
 *         * treat this as an empty log:
 *             g_head_idx = g_tail_idx = 0xFFFFFFFF
 *             g_next_idx = 0
 *             g_next_id  = 0
 *
 *   - If at least one VALID record was found:
 *         * g_tail_idx = index of smallest ID (oldest record)
 *         * g_head_idx = index of largest ID (newest record)
 *         * g_next_id  = max_id + 1
 *         * g_next_idx = (g_head_idx + 1) % FTL_NUM_UNITS
 *
 * Returns:
 *   0  on success
 *  <0 on error (basic IO error handling for now).
 */

int ftl_mount(void)
{
    uint32_t min_id  = 0xFFFFFFFFu;
    uint32_t max_id  = 0u;
    uint32_t min_idx = 0xFFFFFFFFu;
    uint32_t max_idx = 0xFFFFFFFFu;

    // Scan every 4 KB unit in the FTL region
    for (uint32_t i = 0; i < FTL_NUM_UNITS; i++) {
        uint32_t base_addr = ftl_unit_base_addr(i);
        uint32_t hdr_addr  = base_addr + FTL_HEADER_OFFSET;

        ftl_record_header_t hdr;
        memset(&hdr, 0xFF, sizeof(hdr));  // just defensive
        read_data(hdr_addr, (uint8_t *)&hdr, (uint16_t)sizeof(hdr));

        // Case 1: completely erased / never used block
        if (hdr.status == FTL_STATUS_EMPTY && hdr.id == 0xFFFFFFFFu) {
            continue;
        }

        // For now, we only care about blocks marked VALID.
        if (hdr.status != FTL_STATUS_VALID) {
            continue;
        }

        // Basic sanity check on length
        if (hdr.length == 0u || hdr.length > FTL_MAX_PAYLOAD) {
            // Length is nonsense; ignore this header.
            continue;
        }

        // (Optional future step: CRC check of payload bytes.
        //  We'll add that once append/write is in place.)

        // Track oldest (min_id) and newest (max_id) record
        if (hdr.id < min_id) {
            min_id  = hdr.id;
            min_idx = i;
        }
        if (max_idx == 0xFFFFFFFFu || hdr.id > max_id) {
            max_id  = hdr.id;
            max_idx = i;
        }
    }

    if (min_idx == 0xFFFFFFFFu) {
        // No VALID records found at all.
        // This is either a freshly formatted device or fully erased.
        g_head_idx = 0xFFFFFFFFu;
        g_tail_idx = 0xFFFFFFFFu;
        g_next_idx = 0u;  // start writing at unit 0
        g_next_id  = 0u;  // first record will have ID 0
    } else {
        // We found at least one VALID record.
        g_head_idx = max_idx;       // newest
        g_tail_idx = min_idx;       // oldest
        g_next_id  = max_id + 1u;   // ID for the next record

        // Next write goes after 'head', wrapping around
        uint32_t next = g_head_idx + 1u;
        if (next >= FTL_NUM_UNITS) {
            next = 0u;
        }
        g_next_idx = next;
    }

    g_mounted = true;
    return 0;
}

int ftl_read(uint32_t block_id, uint8_t* out, uint16_t* len) {
	if (!g_mounted) return -1;

	uint32_t idx = g_tail_idx;
	ftl_record_header_t header;
	bool id_found = false;
	for (idx = g_tail_idx; idx <= g_head_idx; idx++) {
		memset(&header, 0xFF, sizeof(header));
		read_data(ftl_unit_base_addr(idx), (uint8_t*) &header, sizeof(header));

		if (header.status != FTL_STATUS_VALID) continue;
		if (header.id == block_id) {
			id_found = true;
			break;
		}
	}

	if (!id_found) return -2;

	printf("Header:\r\n");
	printf("  ID:        %lu\r\n", (unsigned long) header.id);
	printf("  Status:    0x%04X\r\n", header.status);
	printf("  Length:    %u\r\n", header.length);
	printf("  CRC32:     0x%08lX\r\n", (unsigned long) header.crc);
	printf("  Reserved:  ");
	for (int i = 0; i < 8; i++) printf("%02X ", header.reserved[i]);
	printf("\r\n");

	read_data(ftl_unit_base_addr(idx) + FTL_HEADER_PAGE_SIZE, out, header.length);
	*len = header.length;

	return 0;
}


// helper function for debugging
ftl_state_view_t ftl_get_state(void)
{
    ftl_state_view_t s;
    s.head_idx = g_head_idx;
    s.tail_idx = g_tail_idx;
    s.next_idx = g_next_idx;
    s.next_id  = g_next_id;
    s.mounted  = g_mounted;
    return s;
}


void test_format(void){
	printf("starting function to test formatting \r\n");
	printf("Formatting...\r\n");
	int r = ftl_format();
	printf("ftl_format() = %d\r\n", r);
}

void test_mount(void){
	printf("starting function to test mounting...\r\n");

	printf("Mounting...\r\n");
	int r = ftl_mount();
	printf("ftl_mount() = %d\r\n", r);

	ftl_state_view_t st = ftl_get_state();
	printf("head=%lu tail=%lu next=%lu next_id=%lu mounted=%d\r\n",
	       (unsigned long)st.head_idx,
	       (unsigned long)st.tail_idx,
	       (unsigned long)st.next_idx,
	       (unsigned long)st.next_id,
	       (int)st.mounted);

}

void test_format_and_mount(void){

	ftl_format();              // ensure everything is erased first

	ftl_record_header_t h;
	memset(&h, 0xFF, sizeof(h));
	h.id     = 2;
	h.status = FTL_STATUS_VALID;
	h.length = 100;
	h.crc    = 0x12345678;

	page_program(0, (uint8_t *)&h, sizeof(h));

	int r = ftl_mount();

	printf("ftl_mount() = %d\r\n", r);
	ftl_state_view_t st = ftl_get_state();
	printf("head=%lu tail=%lu next=%lu next_id=%lu mounted=%d\r\n",
	       (unsigned long)st.head_idx,
	       (unsigned long)st.tail_idx,
	       (unsigned long)st.next_idx,
	       (unsigned long)st.next_id,
	       (int)st.mounted);
	/*
	 * we expect the following output:
	 * head_idx = 0, tail_idx = 0, next_idx = 1, next_id = 3
	 */
}


void test_format_and_mount_two_records(void)
{
    // 1) Ensure flash is clean
    ftl_format();

    /*
     * 2) Create and write FIRST record (index = 0, ID = 2)
     */
    ftl_record_header_t h1;
    memset(&h1, 0xFF, sizeof(h1));
    h1.id     = 2;
    h1.status = FTL_STATUS_VALID;
    h1.length = 50;
    h1.crc    = 0xAAAA5555;   // dummy CRC for now

    // Write header for record at physical block index 0
    page_program(0, (uint8_t *)&h1, sizeof(h1));


    /*
     * 3) Create and write SECOND record (index = 5, ID = 7)
     */
    ftl_record_header_t h2;
    memset(&h2, 0xFF, sizeof(h2));
    h2.id     = 7;
    h2.status = FTL_STATUS_VALID;
    h2.length = 120;
    h2.crc    = 0x1234ABCD;   // dummy CRC again

    // Write header for record at physical block index 5
    // base address = 5 * 4096 = 0x5000 (decimal 20480)
    uint32_t block5_addr = 5 * FTL_UNIT_SIZE;
    page_program(block5_addr, (uint8_t *)&h2, sizeof(h2));


    /*
     * 4) Mount the FTL and examine internal state
     */
    int r = ftl_mount();
    printf("ftl_mount() = %d\r\n", r);

    ftl_state_view_t st = ftl_get_state();
    printf("head=%lu tail=%lu next=%lu next_id=%lu mounted=%d\r\n",
           (unsigned long)st.head_idx,
           (unsigned long)st.tail_idx,
           (unsigned long)st.next_idx,
           (unsigned long)st.next_id,
           (int)st.mounted);

    /*
     * EXPECTED OUTPUT:
     *
     * Written VALID blocks:
     *   index 0 → id = 2
     *   index 5 → id = 7
     *
     * → oldest record  = smallest ID = 2 → tail_idx = 0
     * → newest record  = largest  ID = 7 → head_idx = 5
     *
     * next_idx = (head_idx + 1) % FTL_NUM_UNITS = 6
     * next_id  = max_id + 1 = 8
     *
     * So we expect:
     *   head_idx = 5
     *   tail_idx = 0
     *   next_idx = 6
     *   next_id  = 8
     *   mounted  = 1
     */
}

void test_read(void) {
	printf("Begin Testing\r\n");

	uint32_t id = 1;

	uint8_t buf[FTL_MAX_PAYLOAD + 1];
	uint16_t len;
	int st = ftl_read(id, buf, &len);
	if (st != 0) {
		printf("Read Error: %d\r\n", st);
		return;
	}

	buf[len] = '\0';
	printf("Msg: %s (Length: %d)\r\n", (char*) buf, len);
}

