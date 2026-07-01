#ifndef SD_SPI_H
#define SD_SPI_H

#include <stdint.h>

#include "main.h"
#include "ff.h"
#include "diskio.h"
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi2;

#define SD_SPI_OK 0
#define SD_SPI_ERROR 1

#define SD_SPI_BLOCK_SIZE 512 // Size for FATFS

uint8_t SD_SPI_Init(void); // For user_diskio.h
uint8_t SD_SPI_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
uint8_t SD_SPI_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
uint8_t SD_SPI_Sync(void);
uint32_t SD_SPI_GetSectorCount(void);

#endif //SD_SPI_H