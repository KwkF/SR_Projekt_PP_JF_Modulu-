/*
 * flash.c
 *
 *  Created on: Apr 29, 2024
 *      Author: projectrobal
 */


#include "flash.h"


// write byte to flash at specified address
flash_status_t N25_flash_write(QSPI_HandleTypeDef* qspi,uint8_t* data,uint32_t write_addr, uint32_t size)
{
  QSPI_CommandTypeDef sCommand;
  uint32_t end_addr, current_size, current_addr;

  current_size = N25Q128A_PAGE_SIZE - (write_addr % N25Q128A_PAGE_SIZE);

  if (current_size > size)
  {
	current_size = size;
  }


  current_addr = write_addr;
  end_addr = write_addr + size;


  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = EXT_QUAD_IN_FAST_PROG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_4_LINES;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;


  do
  {
	sCommand.Address = current_addr;
	sCommand.NbData  = current_size;

	if (QSPI_WriteEnable(qspi) != FLASH_OK)
	{
	  return FLASH_ERROR;
	}

	if (HAL_QSPI_Command(qspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	  return FLASH_ERROR;
	}

	if (HAL_QSPI_Transmit(qspi, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
	  return FLASH_ERROR;
	}

	if (QSPI_AutoPollingMemReady(qspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != FLASH_OK)
	{
	  return FLASH_ERROR;
	}

	current_addr += current_size;
	data += current_size;
	current_size = ((current_addr + N25Q128A_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : N25Q128A_PAGE_SIZE;
  }
  while (current_addr < end_addr);

  return FLASH_OK;
}
// read byte from flash at specified address
flash_status_t N25_flash_read(QSPI_HandleTypeDef* qspi,uint8_t* data,uint32_t read_addr, uint32_t size)
{
	QSPI_CommandTypeDef sCommand;

	  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	  sCommand.Instruction       = QUAD_INOUT_FAST_READ_CMD;
	  sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
	  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	  sCommand.Address           = read_addr;
	  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	  sCommand.DataMode          = QSPI_DATA_4_LINES;
	  sCommand.DummyCycles       = N25Q128A_DUMMY_CYCLES_READ_QUAD;
	  sCommand.NbData            = size;
	  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	  if (HAL_QSPI_Command(qspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
		return FLASH_ERROR;
	  }

	  if (HAL_QSPI_Receive(qspi, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
		return FLASH_ERROR;
	  }

	  return FLASH_OK;
}

uint8_t N25_erase_flash(QSPI_HandleTypeDef* qspi)
{

	QSPI_CommandTypeDef sCommand;

	  /* Initialize the erase command */
	  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	  sCommand.Instruction       = BULK_ERASE_CMD;
	  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	  sCommand.DataMode          = QSPI_DATA_NONE;
	  sCommand.DummyCycles       = 0;
	  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	  /* Enable write operations */
	  if (QSPI_WriteEnable(qspi) != FLASH_OK)
	  {
	    return FLASH_ERROR;
	  }

	  /* Send the command */
	  if (HAL_QSPI_Command(qspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
	    return FLASH_ERROR;
	  }

	  /* Configure automatic polling mode to wait for end of erase */
	  if (QSPI_AutoPollingMemReady(qspi, N25Q128A_BULK_ERASE_MAX_TIME) != FLASH_OK)
	  {
	    return FLASH_ERROR;
	  }

	  return FLASH_OK;

}

flash_status_t N25_erase_block(QSPI_HandleTypeDef* qspi,uint32_t block_address)
{
	QSPI_CommandTypeDef sCommand;

	  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	  sCommand.Instruction       = SUBSECTOR_ERASE_CMD;
	  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
	  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	  sCommand.Address           = block_address;
	  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	  sCommand.DataMode          = QSPI_DATA_NONE;
	  sCommand.DummyCycles       = 0;
	  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	  /* Enable write operations */
	  if (QSPI_WriteEnable(qspi) != FLASH_OK)
	  {
		return FLASH_ERROR;
	  }

	  /* Send the command */
	  if (HAL_QSPI_Command(qspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
		return FLASH_ERROR;
	  }

	  /* Configure automatic polling mode to wait for end of erase */
	  if (QSPI_AutoPollingMemReady(qspi, N25Q128A_SUBSECTOR_ERASE_MAX_TIME) != 0)
	  {
		return FLASH_ERROR;
	  }

	  return FLASH_OK;
}

flash_status_t N25_erase_sector(QSPI_HandleTypeDef* qspi,uint32_t sector_address)
{
	QSPI_CommandTypeDef sCommand;

  if (sector_address >= (uint32_t)(N25Q128A_FLASH_SIZE / N25Q128A_SECTOR_SIZE))
  {
	return FLASH_ERROR;
  }

  /* Initialize the erase command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = SECTOR_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.Address           = (sector_address * N25Q128A_SECTOR_SIZE);
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable(qspi) != FLASH_OK)
  {
	return FLASH_ERROR;
  }

  /* Send the command */
  if (HAL_QSPI_Command(qspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
	return FLASH_ERROR;
  }

  return FLASH_OK;
}

flash_status_t N25_flash_status(QSPI_HandleTypeDef* qspi)
{
	QSPI_CommandTypeDef sCommand;
	  uint8_t reg;

	  /* Initialize the read flag status register command */
	  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	  sCommand.Instruction       = READ_FLAG_STATUS_REG_CMD;
	  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	  sCommand.DataMode          = QSPI_DATA_1_LINE;
	  sCommand.DummyCycles       = 0;
	  sCommand.NbData            = 1;
	  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	  /* Configure the command */
	  if (HAL_QSPI_Command(qspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
	    return FLASH_ERROR;
	  }

	  /* Reception of the data */
	  if (HAL_QSPI_Receive(qspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
	    return FLASH_ERROR;
	  }

	  /* Check the value of the register */
	  if ((reg & (N25Q128A_FSR_PRERR | N25Q128A_FSR_VPPERR | N25Q128A_FSR_PGERR | N25Q128A_FSR_ERERR)) != 0)
	  {
	    return FLASH_ERROR;
	  }
	  else if ((reg & (N25Q128A_FSR_PGSUS | N25Q128A_FSR_ERSUS)) != 0)
	  {
	    return FLASH_SUSPENDED;
	  }
	  else if ((reg & N25Q128A_FSR_READY) != 0)
	  {
	    return FLASH_OK;
	  }
	  else
	  {
	    return FLASH_BUSY;
	  }
}
