#include "sd_spi.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi2;

// SPI Commands
#define CMD0 0 // Idle Mode
#define CMD8 8 // Check for SD V2
#define CMD9 9 // Read CSD reg
#define CMD16 16 // Set block length (512 bytes)
#define CMD17 17 // Read block
#define CMD24 24 // Write block
#define CMD55 55 // Next cmd is an application cmd
#define CMD58 58 // Read OCR reg

#define ACMD41 41 // Initialize card

#define SD_DUMMY_BYTE 0xFF // Generate SPI clk without sending data
#define SD_TOKEN_START 0xFE // Start token before reading or writing
#define SD_WRITE_ACCEPTED 0x05 // Write has been accepted

static uint8_t sd_initialized = 0; // Track init status
static uint8_t sd_type_v2 = 0; // Track SD V2 status
static uint8_t sd_type_block_addressing = 0;  // Track if SD uses block addressing

static void SD_Select(void) {
    HAL_GPIO_WritePin(SPI2_CS_PIN_GPIO_Port, SPI2_CS_PIN_Pin, GPIO_PIN_RESET);
}

static void SD_Deselect(void) {
    HAL_GPIO_WritePin(SPI2_CS_PIN_GPIO_Port, SPI2_CS_PIN_Pin, GPIO_PIN_SET);
}

static uint8_t SD_TxRx(uint8_t data) {
    uint8_t rx = 0;
    HAL_StatusTypeDef test = HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, HAL_MAX_DELAY);
    if(test != HAL_OK) {
        return 0xFF;
    }    
    return rx;
}

static void SD_SendDummyClocks(uint8_t bytes) {
    for(uint8_t i = 0; i < bytes; i++){
        SD_TxRx(SD_DUMMY_BYTE);
    }
}

static uint8_t SD_WaitReady(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();

    while((HAL_GetTick() - start) < timeout_ms) {
        if(SD_TxRx(SD_DUMMY_BYTE) == 0xFF) {
            return 1; // Ready
        }
    }

    return 0; // Timed Out
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t response;

    if(!SD_WaitReady(500)) {
        return 0xFF;
    }

    SD_TxRx(0x40 | cmd);
    SD_TxRx((uint8_t)(arg >> 24));
    SD_TxRx((uint8_t)(arg >> 16));
    SD_TxRx((uint8_t)(arg >> 8));
    SD_TxRx((uint8_t)arg);
    SD_TxRx(crc);

    for (uint8_t i = 0; i < 10; i++) {
        response = SD_TxRx(SD_DUMMY_BYTE);
        if ((response & 0x80) == 0) {
            return response;
        }
    }
    return 0xFF;
}

static uint8_t SD_SendACMD(uint8_t acmd, uint32_t arg, uint8_t crc) {
    uint8_t response;
    response = SD_SendCommand(CMD55, 0, 0xFF);
    if(response > 0x01) {
        return response;
    }

    return SD_SendCommand(acmd, arg, crc);
}

static uint8_t SD_ReadCSD(uint8_t *csd) {
    uint8_t response;
    uint8_t token;
    uint32_t start;

    SD_Select();

    response = SD_SendCommand(CMD9, 0, 0xFF);

    if(response != 0x00) {
        SD_Deselect();
        SD_TxRx(SD_DUMMY_BYTE);
        return SD_SPI_ERROR;
    }

    start = HAL_GetTick();

    do {
        token = SD_TxRx(SD_DUMMY_BYTE);
        if(token == SD_TOKEN_START) {
            break;
        }
    } while ((HAL_GetTick() - start) < 200);

    if(token != SD_TOKEN_START) {
        SD_Deselect();
        SD_TxRx(SD_DUMMY_BYTE);
        return SD_SPI_ERROR;
    }

    for(uint8_t i = 0; i < 16; i++) {
        csd[i] = SD_TxRx(SD_DUMMY_BYTE);
    }

    // Ignore CRC
    SD_TxRx(SD_DUMMY_BYTE);
    SD_TxRx(SD_DUMMY_BYTE);

    SD_Deselect();
    SD_TxRx(SD_DUMMY_BYTE);

    return SD_SPI_OK;
}

uint8_t SD_SPI_Init() {
    uint8_t response;
    uint8_t ocr[4];

    sd_initialized = 0;
    sd_type_v2 = 0;
    sd_type_block_addressing = 0;

    SD_Deselect();
    SD_SendDummyClocks(10); // Pull CS high
    SD_Select(); 

    response = SD_SendCommand(CMD0, 0, 0x95); // Reset card and idle

    SD_Deselect();
    SD_TxRx(SD_DUMMY_BYTE);

    if(response != 0x01) {
        return SD_SPI_ERROR;
    }

    SD_Select();
    response = SD_SendCommand(CMD8, 0x000001AA, 0x87);

    if(response == 0x01) {
        sd_type_v2 = 0x01;

        ocr[0] = SD_TxRx(SD_DUMMY_BYTE);
        ocr[1] = SD_TxRx(SD_DUMMY_BYTE);
        ocr[2] = SD_TxRx(SD_DUMMY_BYTE);
        ocr[3] = SD_TxRx(SD_DUMMY_BYTE);

        if (ocr[2] != 0x01 || ocr[3] != 0xAA){
            SD_Deselect();
            SD_TxRx(SD_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }
    }

    SD_Deselect();
    SD_TxRx(SD_DUMMY_BYTE);
    uint32_t start = HAL_GetTick();

    do {
        SD_Select();

        if(sd_type_v2) {
            response = SD_SendACMD(ACMD41, 0x40000000, 0xFF);
        } else {
            response = SD_SendACMD(ACMD41, 0, 0xFF);    
        }

        SD_Deselect();
        SD_TxRx(SD_DUMMY_BYTE);

        if(response == 0x00) {
            break;
        }
        HAL_Delay(1);
    } while ((HAL_GetTick() - start) < 1000);

    if(response != 0x00) {
        SD_Deselect();
        SD_TxRx(SD_DUMMY_BYTE);
        return SD_SPI_ERROR;
    }

    SD_Select();
    response = SD_SendCommand(CMD58, 0, 0xFF);

    if(response == 0x00) {
        ocr[0] = SD_TxRx(SD_DUMMY_BYTE);
        ocr[1] = SD_TxRx(SD_DUMMY_BYTE);
        ocr[2] = SD_TxRx(SD_DUMMY_BYTE);
        ocr[3] = SD_TxRx(SD_DUMMY_BYTE);

        if(ocr[0] & 0x40){
            sd_type_block_addressing = 1;
        }
    }

    SD_Deselect();
    SD_TxRx(SD_DUMMY_BYTE);

    if(!sd_type_block_addressing) {
        SD_Select();
        response = SD_SendCommand(CMD16, 512, 0xFF);
        
        SD_Deselect();
        SD_TxRx(SD_DUMMY_BYTE);

        if(response != 0x00) {
            return SD_SPI_ERROR;
        }
    }

    sd_initialized = 1;
    return SD_SPI_OK;
}

uint8_t SD_SPI_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count) {
    if(!sd_initialized || buff == 0 || count == 0) {
        return SD_SPI_ERROR;
    }

    for(uint32_t i = 0; i < count; i++) {
        uint32_t address = sector + i;

        if(!sd_type_block_addressing) {
            address *= 512;
        }

        SD_Select();
        uint8_t response = SD_SendCommand(CMD17, address, 0xFF);

        if(response != 0x00) {
            SD_Deselect();
            SD_TxRx(SD_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        uint32_t start = HAL_GetTick();
        uint8_t token;

        do {
            token = SD_TxRx(SD_DUMMY_BYTE);
            if(token == SD_TOKEN_START) {
                break;
            }
        } while((HAL_GetTick() - start) < 200);

        if(token != SD_TOKEN_START) {
            SD_Deselect();
            SD_TxRx(SD_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        for(uint16_t j = 0; j <512; j++) {
            buff[(i * 512) + j] = SD_TxRx(SD_DUMMY_BYTE);
        }

        // Skip CRC
        SD_TxRx(SD_DUMMY_BYTE);
        SD_TxRx(SD_DUMMY_BYTE);

        SD_Deselect();
        SD_TxRx(SD_DUMMY_BYTE);
    }
    return SD_SPI_OK;
}

uint8_t SD_SPI_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count) {
    if(!sd_initialized || buff == 0 || count == 0) {
        return SD_SPI_ERROR;
    }

    for(uint32_t i = 0; i < count; i++) {
        uint32_t address = sector + i;

        if(!sd_type_block_addressing) {
            address *= 512;
        }

        SD_Select();
        uint8_t response = SD_SendCommand(CMD24, address, 0xFF);

        if(response != 0x00) {
            SD_Deselect();
            SD_TxRx(SD_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        SD_TxRx(SD_DUMMY_BYTE);
        SD_TxRx(SD_TOKEN_START);

        for(uint16_t j = 0; j < 512; j++) {
            SD_TxRx(buff[(i * 512) + j]);
        }

        SD_TxRx(SD_DUMMY_BYTE);
        SD_TxRx(SD_DUMMY_BYTE);

        response = SD_TxRx(SD_DUMMY_BYTE);

        if((response & 0x1F) != SD_WRITE_ACCEPTED) {
            SD_Deselect();
            SD_TxRx(SD_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        if(!SD_WaitReady(500)) {
            SD_Deselect();
            SD_TxRx(SD_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        SD_Deselect();
        SD_TxRx(SD_DUMMY_BYTE);
    }
    return SD_SPI_OK;
}

uint8_t SD_SPI_Sync() {
    uint8_t ready;

    if(!sd_initialized) {
        return SD_SPI_ERROR;
    }

    SD_Select();

    ready = SD_WaitReady(500);

    SD_Deselect();
    SD_TxRx(SD_DUMMY_BYTE);

    if(ready){
        return SD_SPI_OK;
    }

    return SD_SPI_ERROR;
}

uint32_t SD_SPI_GetSectorCount() {
    uint8_t csd[16];

    if(!sd_initialized || SD_ReadCSD(csd) != SD_SPI_OK) {
        return 0;
    }

    uint8_t csd_structure = (csd[0] >> 6) & 0x03;

    if(csd_structure == 1) {
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | ((uint32_t)csd[9]);
        return (c_size + 1) * 1024;
    } else if (csd_structure == 0) {
        uint32_t c_size = ((uint32_t)(csd[6] & 0x3F) << 10) | ((uint32_t)csd[7] << 2) | ((uint32_t)(csd[8] >> 6));
        uint8_t read_bl_len = csd[5] & 0x0F;
        uint8_t c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7);

        uint32_t block_len = 1UL << read_bl_len;
        uint32_t mult = 1UL << (c_size_mult + 2);
        uint32_t block_count = (c_size + 1) * mult;

        uint32_t capacity_bytes = block_count * block_len;

        return capacity_bytes / 512;
    }
    return 0;
}