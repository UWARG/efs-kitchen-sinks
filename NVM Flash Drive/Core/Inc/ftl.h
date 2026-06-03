#ifndef INC_FTL_H_
#define INC_FTL_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stm32l5xx_hal.h"
#include "stm32l5xx_nucleo.h"
#include <stdio.h>
#include "nvm_driver.h"


/*
 * ============================
 *  FLASH / FTL GEOMETRY
 * ============================
 *
 * Physical device (e.g., N25Q256A):
 *   - Total size:   32 MB
 *   - Erase unit:   4 KB "subsector"
 *   - Page size:    256 B (minimum program unit)
 *
 * FTL layout (NO METADATA / SUPERBLOCK):
 *   - The ENTIRE flash area used by the FTL is treated as a circular buffer.
 *   - Each 4 KB subsector is one "block" / "unit" of the circular buffer.
 *
 * Inside each 4 KB unit:
 *   - FIRST 256-BYTE PAGE is RESERVED ENTIRELY for the BLOCK HEADER.
 *   - PAYLOAD STARTS AT OFFSET 256 (start of second page).
 *
 * That means in each 4 KB unit:
 *   - header:  [0 .. 255]
 *   - payload: [256 .. 4095]
 *
 * There is NO dedicated superblock with metadata; everything is inferred
 * at mount time by scanning headers across all units.
 *
*/

// NVM Chip Parameters
#define FLASH_TOTAL_SIZE_BYTES      ((32u) * (1024u) * (1024u))  // 32 MB total = 33 554 432 bytes
#define FLASH_SECTOR_SIZE_4K        4096u   // erase size in bytes
#define FLASH_PAGE_SIZE_256         256u    // page program size in bytes

// FTL geometry
#define FTL_UNIT_SIZE               FLASH_SECTOR_SIZE_4K   // 4 KB per unit
#define FTL_HEADER_PAGE_SIZE        FLASH_PAGE_SIZE_256		// First page (256 B) of each unit is the header page
#define FTL_HEADER_OFFSET           0u
#define FTL_PAYLOAD_OFFSET          FTL_HEADER_PAGE_SIZE	// Payload starts at the beginning of the second page
#define FTL_MAX_PAYLOAD             (FTL_UNIT_SIZE - FTL_HEADER_PAGE_SIZE)	// Max payload per unit: everything after the first 256 B page
#define FTL_NUM_UNITS               (FLASH_TOTAL_SIZE_BYTES / FTL_UNIT_SIZE)	 // Number of usable 4 KB units in the flash

#define FTL_INVALID_PAGE 0xFFFFFFFFu

#define FTL_CRC_POLY 0xEDB88320

/*
 * ============================
 *  RECORD HEADER FORMAT
 * ============================
 *
 * Each 4 KB unit starts with a 256 B page reserved for the header.
 * We place this struct at offset 0 of that page:
 *   physical address = unit_index * FTL_UNIT_SIZE + FTL_HEADER_OFFSET
 *
 * NOTE: sizeof(ftl_record_header_t) MUST BE <= 256,
 *       so it fits entirely within the first page.
 */

typedef struct __attribute__((packed)) {	// use packed so that no padding is inserted by the compiler
    uint32_t id;        // Monotonically increasing record ID (for ordering).
    uint16_t status;    // Lifecycle status (see FTL_STATUS_* below).
    uint16_t length;    // Payload length in bytes (0 < length <= FTL_MAX_PAYLOAD).
    uint32_t crc;       // CRC32 over the payload bytes.
    uint8_t  reserved[8]; // Reserved for future use (timestamps, logical addr, etc.).
} ftl_record_header_t;



// =========== status values
#define FTL_STATUS_EMPTY   0xFFFFu  // Erased / never written (all 0xFF in header).
#define FTL_STATUS_VALID   0x3FFFu  // Fully written and CRC-verified at write time.
#define FTL_STATUS_STALE   0x1FFFu  // Logically obsolete (superseded by newer data).
#define FTL_STATUS_BAD     0x0FFFu  // Known-bad record (CRC fail, header nonsense, etc.).



// Waits

#define FTL_MAX_WAIT ((uint32_t)60000u)



// struct and function used for debugging and printing ftl globals
typedef struct {
    uint32_t head_idx;
    uint32_t tail_idx;
    uint32_t next_idx;
    uint32_t next_id;
    bool     mounted;
} ftl_state_view_t;

ftl_state_view_t ftl_get_state(void);

extern uint32_t g_head_idx;
extern uint32_t g_tail_idx;
extern uint32_t g_next_idx;
extern uint32_t g_next_id;
extern bool     g_mounted;


/* basic FTL API prototypes ==== */
/* 0 = success, <0 = error for now. replace int with an enum later. */
#ifdef __cplusplus
enum class MessageType {
	BATTERY_STATUS = 0,
};

class AbstractMessage {
public:
	uint32_t id;

	AbstractMessage(uint32_t id);
	virtual ~AbstractMessage() = default;

	virtual int unpack(const uint8_t* data, uint16_t len) = 0;
	virtual int pack(uint8_t* data, uint16_t& len) = 0;

	virtual uint16_t packed_size() const = 0;
};

class BatteryLog : public AbstractMessage {
public:
	uint16_t voltage;
	uint16_t current;
	uint16_t power;

	static const uint16_t PACKED_SIZE = sizeof(uint16_t) * 3;

	BatteryLog(uint32_t id, uint16_t voltage, uint16_t current, uint16_t power);

	int unpack(const uint8_t* data, uint16_t len) override;
	int pack(uint8_t* data, uint16_t& len) override;

	uint16_t packed_size() const override { return PACKED_SIZE; };
};

class FTL
{
	protected:
		FTL();

	public:
		static FTL& get_instance();

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
		int format(void);

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

		int mount(void);

		/*
		 * ftl_write
		 *
		 * Write a amount of data
		 * maximum amount of data is
		 *
		 * Steps:
		 *   - Use g_next_idx as the physical block to write into.
		 *   - Use g_next_id as the record ID to store in the header.
		 *   - Erase the entire 4 KB unit before writing (always required for reuse).
		 *   - Write the header into the first 256-byte page.
		 *   - Write the payload starting at FTL_PAYLOAD_OFFSET.
		 *   - Update FTL state:
		 *        head_idx = g_next_idx
		 *        if overwriting tail: advance tail_idx
		 *        next_idx = (g_next_idx + 1) % FTL_NUM_UNITS
		 *        next_id  = g_next_id + 1
		 *
		 * Each block stores exactly one immutable record.
		 * Updating requires writing a new record, not modifying the old one.
		 *
		 * Returns 0 on success, <0 on error.
		 */

		int write(AbstractMessage* data);

		/**
		 * ftl_read
		 *
		 * Reads a block of data according to its ID
		 *
		 * Steps:
		 * 	- Scanning for the block ID in each block's header
		 * 	- Starts from g_tail_idx
		 * 	- Ends at g_head_idx, if this is reached, no valid block was found
		 * 	- Once a valid block with the correct block ID is found, stop searching
		 * 	- With the correct block, read the payload using header.length
		 * 	- Copy the payload to user-provided output buffer
		 * 	- Return the payload length
		 *
		 * Each block's CRC is not checked
		 *
		 * @return
		 *   0		on success
		 *   -1		if FTL is not mounted
		 *   -2		if no valid record for the specified block ID could be found
		 */
		int read(AbstractMessage* out);

		//erase function
		int erase(AbstractMessage* msg);

		int update(AbstractMessage* msg);

		// Erase all FTL units (destroys all data).
		int ftl_format(void);

		// Scan all units, find oldest/newest VALID records, init internal FTL state.
		int ftl_mount(void);

		int ftl_read(uint32_t block_id, uint8_t* out, uint16_t* len);

		// write one record (1 record per 4KB unit). Returns 0 on success.
		int ftl_write(const void *data, uint16_t len, uint32_t *out_id);


		// testing functions
		void test_format(void);
		void test_mount(void);
		void test_format_and_mount(void);
		void test_format_and_mount_two_records(void);
		void test_write_and_read_latest(void);

		void test_read(void);

		// helper function for debugging
		ftl_state_view_t get_state(void);

		void test_message(BatteryLog log);


		/*
		 * - uint8_t *data: data array
		 * - int len: the length of input data array
		 * returns LSB-first (reflected) CRC result.
		 */
		static uint32_t crc32(const uint8_t *data, uint32_t len);


	private:
		uint32_t head_idx  = 0xFFFFFFFFu;
		uint32_t tail_idx  = 0xFFFFFFFFu;
		uint32_t next_idx  = 0u;
		uint32_t next_id   = 0u;
		bool     mounted   = false;

		// Helper function to compute the base address in flash for a given unit index
		inline uint32_t unit_base_addr(uint32_t unit_index);


};

#endif


// Erase all FTL units (destroys all data).
int ftl_format(void);

// Scan all units, find oldest/newest VALID records, init internal FTL state.
int ftl_mount(void);

int ftl_read(uint32_t block_id, uint8_t* out, uint16_t* len);

// write one record (1 record per 4KB unit). Returns 0 on success.
int ftl_write(const void *data, uint16_t len, uint32_t *out_id);

int ftl_erase(uint32_t block_id);

int ftl_update(uint32_t block_id, const void* data, uint16_t len);


// testing functions
void test_format(void);
void test_mount(void);
void test_format_and_mount(void);
void test_format_and_mount_two_records(void);
void test_write_and_read_latest(void);

void test_read(void);



extern SPI_HandleTypeDef hspi1;


#endif /* INC_FTL_H_ */
