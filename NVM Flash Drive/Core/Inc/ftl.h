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



// struct and function used for debugging and printing ftl globals
typedef struct {
    uint32_t head_idx;
    uint32_t tail_idx;
    uint32_t next_idx;
    uint32_t next_id;
    bool     mounted;
} ftl_state_view_t;

ftl_state_view_t ftl_get_state(void);


/* basic FTL API prototypes ==== */
/* 0 = success, <0 = error for now. replace int with an enum later. */

// Erase all FTL units (destroys all data).
int ftl_format(void);

// Scan all units, find oldest/newest VALID records, init internal FTL state.
int ftl_mount(void);

int ftl_read(uint32_t block_id, uint8_t* out, uint16_t* len);

void test_format(void);
void test_mount(void);
void test_format_and_mount(void);
void test_format_and_mount_two_records(void);

void test_read(void);

extern SPI_HandleTypeDef hspi1;


#endif /* INC_FTL_H_ */
