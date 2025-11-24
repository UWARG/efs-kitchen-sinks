#include "nvm_driver.h"

static inline void CS_LOW(void)  { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET); }
static inline void CS_HIGH(void) { HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);   }


// low-level helper functions
static inline void spi_tx1(uint8_t v) { HAL_SPI_Transmit(&hspi1, &v, 1, 1000);}
static inline void spi_tx(const uint8_t *buf, uint16_t n) { HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, n, 1000);}
static void spi_rx(uint8_t *buf, uint16_t n) { HAL_SPI_Receive(&hspi1, buf, n, 1000);}

// function to read the ID of the NVM chip with command 9f
HAL_StatusTypeDef readID(uint8_t id[20]){
	// read the ID
	uint8_t id_cmd = 0x9f;
	HAL_StatusTypeDef stat;
	CS_LOW();
	stat = HAL_SPI_Transmit(&hspi1, &id_cmd, 1, 100);
	if (stat == HAL_OK){
		stat = HAL_SPI_Receive(&hspi1, id, 20, 1000);
	}
	CS_HIGH();
	if (stat == HAL_OK) {
		// we expoect ID to be 0x20, 0xBA, 0x19
		printf("JEDEC: %02X %02X %02X\r\n", id[0], id[1], id[2]);    // id[0] = MFR, id[1] = TYPE, id[2] = CAP
	}
	else printf("coudn't get ID, error \r\n");
	return stat;
}

/*
 * function to read status register
 * bit0 (LSB) = write in progress: 1 when busy (write, program, or erase in progress), 0 when ready
 * bit1 = write latch, 0 when cleared, 1 when set
 */
uint8_t read_sr(void) {
    uint8_t sr = 0, cmd = READ_STATUS;
    CS_LOW();
    spi_tx1(cmd);
    spi_rx(&sr, 1);
    CS_HIGH();
    return sr;
}

/*
 * function to read the flag status register
 * bit 8 (MSB) = status bit, 1 means ready, 0 means busy
 * this is different from the read status register, it gives more error codes
 */
uint8_t read_fsr(void) {
    uint8_t fsr = 0, cmd = FLAG_STATUS;
    CS_LOW();
    spi_tx1(cmd);
    spi_rx(&fsr, 1);
    CS_HIGH();
    return fsr;
}

/*
 * function to clear the flag status register
 * when run into an error and want to try again, good to use this
 */
void clear_fsr(void) {
    uint8_t cmd = CLEAR_FLAG;
    CS_LOW(); spi_tx1(cmd); CS_HIGH();
}

/*
 * function to enable writes, should call this before every write
 * returns true = write is enabled
 */
bool write_enable(void) {
    uint8_t cmd = WRITE_ENABLE;
    CS_LOW(); spi_tx1(cmd); CS_HIGH();
    // make sure that the write enable latch is set (bit 1 of the status register)
    return (read_sr() & 0x02) != 0;
}

/*
 * function to wait a specific time and keep checking the SR register
 * use this to poll status reg after performing a read/write/erase
 */
HAL_StatusTypeDef wait_ready(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while(1) {
        if ((read_sr() & 0x01) == 0) {	// finished when bit 0 of status reg is reset
            uint8_t fsr = read_fsr();
            if (fsr & 0x60) { // erase/program error
                clear_fsr();
                return HAL_ERROR;
            }
            return HAL_OK;
        }
        if ((HAL_GetTick() - t0) > timeout_ms) return HAL_TIMEOUT;
    }
}

/*
 * function to erase a 4K sector
 * make sure to enable write before calling this
 * the least significant 12 bits of add24 will be ignored (need multiples of 4096)
 */
HAL_StatusTypeDef erase_4k(uint32_t addr24) {
    if (!write_enable()) return HAL_ERROR;
    uint8_t cmd[4] = { SUBSECTOR_ERASE_4K,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    CS_LOW(); spi_tx(cmd, sizeof(cmd)); CS_HIGH();
    return wait_ready(400); // should be plenty for 4KB erase
}

HAL_StatusTypeDef erase_full() {
	if (!write_enable()) return HAL_ERROR;

	uint8_t cmd[1] = { 0xC7 };

	CS_LOW(); spi_tx(cmd, sizeof(cmd)); CS_HIGH();
	return wait_ready(231000); // Maximum 256 Mb bulk erase time
}

/*
 * function to program data into an array
 * the 24 bit address can be anywhere in the range, but there are page alignment rules
 * no automatic wrap into the next page
 */
HAL_StatusTypeDef page_program(uint32_t addr24, const uint8_t *data, uint16_t len) {
    if (len == 0 || len > 256) return HAL_ERROR;
    if (!write_enable()) return HAL_ERROR;

    // check to make sure alignment is correct (write at the start of a page)
    if (((addr24 & 0xFF) + len) > 256 ) return HAL_ERROR;

    uint8_t hdr[4] = { PAGE_PROGRAM,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    CS_LOW();
    spi_tx(hdr, sizeof(hdr));
    spi_tx(data, len);
    CS_HIGH();
    return wait_ready(10); // page program typically <1ms; 10ms guard
}

/*
 * function to read data
 * addr24 is where the data will start from, and increment
 */
void read_data(uint32_t addr24, uint8_t *out, uint16_t len) {
    uint8_t hdr[4] = { READ,
                       (uint8_t)(addr24 >> 16),
                       (uint8_t)(addr24 >> 8),
                       (uint8_t)(addr24) };
    CS_LOW();
    spi_tx(hdr, sizeof(hdr));   // send cmd+addr
    spi_rx(out, len);           // then clock out data
    CS_HIGH();
}

void ReadTest(void) {
    const uint32_t addr = 0x00000008; // 3B address
    const uint8_t tx[200] = {0xAA};

    uint8_t rx[200] = {0};
    uint32_t base4k = addr & ~0x0FFFU;

    // ---- READ BACK ----
    read_data(addr, rx, sizeof(rx));
    printf("data is ===================================== \r\n");
    for (int i = 0; i < 200; i++){
    	printf("%2X\r\n", tx[i]);
    }
    printf("done printing data ===================================== \r\n");

    // ---- VERIFY ----
    if (memcmp(tx, rx, sizeof(tx)) == 0) {
        printf("PASS: data verified at 0x%06lX\n", (unsigned long)addr);
    } else {
        printf("FAIL: mismatch at 0x%06lX\n", (unsigned long)addr);
    }

    printf("END OF TEST FUNCTION\r\n");
    return;
}



void WriteTest(void) {
    const uint32_t addr = 0x00000008; // 3B address
    const uint8_t tx[200] = {0xAA};

    uint8_t rx[200] = {0};
    uint32_t base4k = addr & ~0x0FFFU;

    // ---------------- ERASE ---------------------
    if (!write_enable()) {
         printf("write enable failed before erase\r\n");
        return;
    }
    if (erase_4k(base4k) != HAL_OK) {         // send 0x20, 3B address
         printf("unable to erase subsector\r\n");
        return;
    }
    if (wait_ready(3000) != HAL_OK) {         // poll SR.WIP=0, timeout
         printf("didn't receive ready after erase\r\n");
        return;
    }


    // ------------------- WRITE  ---------------------

    if (!write_enable()) {
         printf("unable to enable write after erasing\r\n");
        return;
    }

    if (page_program(addr, tx, sizeof(tx)) != HAL_OK) { // send 0x02 + data
         printf("unable to write to page \r\n");
        return;
    }

    if (wait_ready(10) != HAL_OK) {           // poll until program finishes
         printf("timeout after %d ms \r\n", 10);
        return;
    }
}


/*
 * function to test writing data, then reading it back.
 * should do this with a power cycle as well
 */
void ReceiveTransmitTest(void)
{
    const uint32_t addr = 0x00000008; // 3B address
    const uint8_t tx[200] = {0xAA};

    uint8_t rx[200] = {0};
    uint32_t base4k = addr & ~0x0FFFU;


    // ---------------- ERASE ---------------------
    if (!write_enable()) {
         printf("write enable failed before erase\r\n");
        return;
    }
    if (erase_4k(base4k) != HAL_OK) {         // send 0x20, 3B address
         printf("unable to erase subsector\r\n");
        return;
    }
    if (wait_ready(3000) != HAL_OK) {         // poll SR.WIP=0, timeout
         printf("didn't receive ready after erase\r\n");
        return;
    }


    // ------------------- WRITE  ---------------------

    if (!write_enable()) {
         printf("unable to enable write after erasing\r\n");
        return;
    }

    if (page_program(addr, tx, sizeof(tx)) != HAL_OK) { // send 0x02 + data
         printf("unable to write to page \r\n");
        return;
    }

    if (wait_ready(10) != HAL_OK) {           // poll until program finishes
         printf("timeout after %d ms \r\n", 10);
        return;
    }

    // ---- READ BACK ----
    read_data(addr, rx, sizeof(rx));

    // ---- VERIFY ----
    if (memcmp(tx, rx, sizeof(tx)) == 0) {
        printf("PASS: data verified at 0x%06lX\n", (unsigned long)addr);
    } else {
        printf("FAIL: mismatch at 0x%06lX\n", (unsigned long)addr);
    }

    printf("END OF TEST FUNCTION\r\n");
    return;
}


void txTest() {
	uint8_t address[3] = {0x00, 0x00, 0x01};
	uint8_t* write_cmd = malloc(4 * sizeof(uint8_t));		// shouldn't use malloc (also need free), because size is known
	write_cmd[0] = WRITE;
	memcpy(write_cmd + 1, address, 3 * sizeof(uint8_t));
	uint8_t* read_cmd = malloc(4 * sizeof(uint8_t));
	read_cmd[0] = READ;
	memcpy(read_cmd + 1, address, 3 * sizeof(uint8_t));

	uint8_t tx[4] = {0xA6, 0x13, 0x15, 0x1B};
	uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);		// much easier to use CS_LOW to clean things up
	HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_ENABLE, 1, HAL_MAX_DELAY);		// after transmitting this, should check status reg
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, write_cmd, 4, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_DISABLE, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


	// in this while loop, the CS pin shound't be set and reset each time, that could interrupt any operations
	// CS only needs to go high between commands
	// all that needs to be done is polling.

	uint8_t status;
	do {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) READ_STATUS, &status, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_Delay(100);
	} while (status & 0x01);

	HAL_Delay(300);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, read_cmd, rx, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


void rxTest() {
	uint8_t address[3] = {0x00, 0x00, 0x01};

	#ifdef RX_TX
		uint8_t tx[4] = {0xA6, 0x13, 0x15, 0x1B};
		uint8_t* write_cmd = malloc(4 * sizeof(uint8_t));
		write_cmd[0] = WRITE;
		memcpy(write_cmd + 1, address, 3 * sizeof(uint8_t));

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_ENABLE, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, write_cmd, 4, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) WRITE_DISABLE, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		uint8_t status;
		do {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) READ_STATUS, &status, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_Delay(100);
		} while (status & 0x01);
	#endif

	#ifdef RX_RX
		uint8_t* read_cmd = malloc(4 * sizeof(uint8_t));
		read_cmd[0] = READ;
		memcpy(read_cmd + 1, address, 3 * sizeof(uint8_t));
		uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, read_cmd, rx, 4, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	#endif
}



