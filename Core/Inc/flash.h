/*
 * flash.h
 *
 *  Created on: Apr 29, 2024
 *      Author: projectrobal
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_


#include <stm32l4xx_hal.h>
#include <n25q128a.h>

typedef enum flash_status
{
	FLASH_OK=0,
	FLASH_ERROR,
	FLASH_BUSY,
	FLASH_NOT_SUPPORTED,
	FLASH_SUSPENDED
} flash_status_t;

// write byte to flash at specified address
flash_status_t N25_flash_write(QSPI_HandleTypeDef* qspi,uint8_t* data,uint32_t write_addr, uint32_t size);
// read byte from flash at specified address
flash_status_t N25_flash_read(QSPI_HandleTypeDef* qspi,uint8_t* data,uint32_t read_addr, uint32_t size);

flash_status_t N25_erase_flash(QSPI_HandleTypeDef* qspi);

flash_status_t N25_erase_block(QSPI_HandleTypeDef* qspi,uint32_t block_address);

flash_status_t N25_erase_sector(QSPI_HandleTypeDef* qspi,uint32_t sector_address);

flash_status_t N25_flash_status(QSPI_HandleTypeDef* qspi);

static uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef* hqspi);

static uint8_t QSPI_DummyCyclesCfg(QSPI_HandleTypeDef* hqspi);

static uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef* hqspi);

static uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef* hqspi, uint32_t Timeout);


#endif /* INC_FLASH_H_ */
