#ifndef INC_NVM_DRIVER_H_
#define INC_NVM_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "stm32l5xx_hal.h"
#include "stm32l5xx_nucleo.h"
#include <stdio.h>


#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04
#define WRITE 0x02
#define READ 0x03
#define READ_STATUS 0x05
#define PAGE_PROGRAM   0x02
#define FLAG_STATUS 0x70
#define CLEAR_FLAG 0x50
#define SUBSECTOR_ERASE_4K 0x20
#define FLASH_CS_PORT GPIOA
#define FLASH_CS_PIN  GPIO_PIN_4

#define RX_TX
#define RX_RX


// testing functions, use these in main
HAL_StatusTypeDef readID(uint8_t id[20]);
void ReadTest(void);
void WriteTest(void);
void ReceiveTransmitTest(void);
void txTest();

// static functions for reading and writing
uint8_t read_sr(void);
uint8_t read_fsr(void);
void clear_fsr(void);
bool write_enable(void);
HAL_StatusTypeDef wait_ready(uint32_t timeout_ms);
HAL_StatusTypeDef erase_4k(uint32_t addr24);
HAL_StatusTypeDef erase_full();
HAL_StatusTypeDef page_program(uint32_t addr24, const uint8_t *data, uint16_t len);
void read_data(uint32_t addr24, uint8_t *out, uint16_t len);


extern SPI_HandleTypeDef hspi1;


#endif /* INC_NVM_DRIVER_H_ */
