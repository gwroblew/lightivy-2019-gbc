/**
 * \file
 *
 * \brief Common SPI interface for SD/MMC stack
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#define UNUSED(x)
#define Assert(x)

#include "../system.h"
#include <string.h>
//#include "driver_init.h"
#include "conf_sd_mmc.h"
#include "sd_mmc_protocol.h"
#include "sd_mmc_spi.h"

#ifdef SD_MMC_SPI_MODE

/**
 * \ingroup sd_mmc_stack_spi
 * \defgroup sd_mmc_stack_spi_internal Common SPI interface for SD/MMC stack
 * implementation
 *
 * @{
 */

// Enable debug information for SD/MMC SPI module
#ifdef SD_MMC_SPI_DEBUG
#include <stdio.h>
#define sd_mmc_spi_debug(...)      printf(__VA_ARGS__)
#else
#define sd_mmc_spi_debug(...)
#endif

// Check configurations
#if (!defined SD_MMC_SPI_MEM_CNT) || (SD_MMC_SPI_MEM_CNT == 0)
#  warning SD_MMC_SPI_MEM_CNT must be defined in conf_sd_mmc.h file.
#  define SD_MMC_SPI_MEM_CNT 1
#endif

//! Internal global error status
static sd_mmc_spi_errno_t IRDATA sd_mmc_spi_err;

//! 32 bits response of the last command
static uint32_t IRDATA sd_mmc_spi_response_32;
//! Current position (byte) of the transfer started by mci_adtc_start()
static uint32_t IRDATA sd_mmc_spi_transfert_pos;
//! Size block requested by last mci_adtc_start()
static uint32_t IRDATA sd_mmc_spi_block_size;
//! Total number of block requested by last mci_adtc_start()
static uint32_t IRDATA sd_mmc_spi_nb_block;

static uint8_t sd_mmc_spi_crc7(uint8_t * buf, uint8_t size);
static bool sd_mmc_spi_wait_busy(void);
static bool sd_mmc_spi_start_read_block(void);
static void sd_mmc_spi_stop_read_block(void);
static void sd_mmc_spi_start_write_block(void);
static bool sd_mmc_spi_stop_write_block(void);
static bool sd_mmc_spi_stop_multiwrite_block(void);

#ifdef  BITBANG_SPI
void ROCODE sendbytes(uint8_t *data, int len) {
  int i, j;
  for (i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (j = 128; j != 0; j = j >> 1) {
      gpio_set_pin_level(SD_MOSI, (b & j) ? 1 : 0);
      //delay_us(1);
      gpio_set_pin_level(SD_SCK, 0);
      //delay_us(1);
      gpio_set_pin_level(SD_SCK, 1);
      //delay_us(1);
    }
  }
}

void ROCODE readbytes(uint8_t *buf, int len) {
  int i, j;
  for (i = 0; i < len; i++) {
    uint8_t rb = 0;
    for (j = 0; j < 8; j++) {
      gpio_set_pin_level(SD_SCK, 0);
      //delay_us(1);
      gpio_set_pin_level(SD_SCK, 1);
      //delay_us(1);
      rb = (rb << 1) | gpio_get_pin_level(SD_MISO);
    }
    buf[i] = rb;
  }
}

#else

void ROCODE sendbytes(uint8_t *data, int len) {
  while (len > 63) {
    oc_spi_send_bytes64((uint32_t *)data);
    data += 64;
    len -= 64;
  }
  while (len--)
    oc_spi_send_byte(*data++);
}

void ROCODE readbytes(uint8_t *buf, int len) {
  while (len > 63) {
    oc_spi_read_bytes64((uint32_t *)buf);
    buf += 64;
    len -= 64;
  }
  while (len--)
    *buf++ = oc_spi_read_byte();
}
#endif

/**
 * \brief Calculates the CRC7
 *
 * \param buf     Buffer data to compute
 * \param size    Size of buffer data
 *
 * \return CRC7 computed
 */
static uint8_t ROCODE sd_mmc_spi_crc7(uint8_t * buf, uint8_t size)
{
	uint8_t crc, value, i;

	crc = 0;
	while (size--) {
		value = *buf++;
		for (i = 0; i < 8; i++) {
			crc <<= 1;
			if ((value & 0x80) ^ (crc & 0x80)) {
				crc ^= 0x09;
			}
			value <<= 1;
		}
	}
	crc = (crc << 1) | 1;
	return crc;
}

/**
 * \brief Wait the end of busy on DAT0 line
 *
 * \return true if success, otherwise false
 */
static bool ROCODE sd_mmc_spi_wait_busy(void)
{
	uint8_t line = 0xFF;

	/* Delay before check busy
	 * Nbr timing minimum = 8 cylces
	 */
	readbytes(&line, 1);

	/* Wait end of busy signal
	 * Nec timing: 0 to unlimited
	 * However a timeout is used.
	 * 200 000 * 8 cycles
	 */
	uint32_t nec_timeout = 200000;
	readbytes(&line, 1);
	do {
		readbytes(&line, 1);
		if (!(nec_timeout--)) {
			return false;
		}
	} while (line != 0xFF);
	return true;
}

/**
 * \brief Sends the correct TOKEN on the line to start a read block transfer
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_spi_err.
 */
static bool ROCODE sd_mmc_spi_start_read_block(void)
{
	uint32_t i;
	uint8_t token;

	Assert(!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size));

	/* Wait for start data token:
	 * The read timeout is the Nac timing.
	 * Nac must be computed trough CSD values,
	 * or it is 100ms for SDHC / SDXC
	 * Compute the maximum timeout:
	 * Frequency maximum = 25MHz
	 * 1 byte = 8 cycles
	 * 100ms = 312500 x spi_read_buffer_wait() maximum
	 */
	token = 0;
	i = 500000;
	do {
		if (i-- == 0) {
			sd_mmc_spi_err = SD_MMC_SPI_ERR_READ_TIMEOUT;
			sd_mmc_spi_debug("%s: Read blocks timeout\n\r", __func__);
			return false;
		}
		readbytes(&token, 1);
		if (SPI_TOKEN_DATA_ERROR_VALID(token)) {
			Assert(SPI_TOKEN_DATA_ERROR_ERRORS & token);
			if (token & (SPI_TOKEN_DATA_ERROR_ERROR
					| SPI_TOKEN_DATA_ERROR_ECC_ERROR
					| SPI_TOKEN_DATA_ERROR_CC_ERROR)) {
				sd_mmc_spi_debug("%s: CRC data error token\n\r", __func__);
				sd_mmc_spi_err = SD_MMC_SPI_ERR_READ_CRC;
			} else {
				sd_mmc_spi_debug("%s: Out of range data error token\n\r", __func__);
				sd_mmc_spi_err = SD_MMC_SPI_ERR_OUT_OF_RANGE;
			}
			return false;
		}
	} while (token != SPI_TOKEN_SINGLE_MULTI_READ);

	return true;
}

/**
 * \brief Executed the end of a read block transfer
 */
static void ROCODE sd_mmc_spi_stop_read_block(void)
{
	uint8_t crc[2];
	// Read 16-bit CRC (not cheked)
	readbytes(crc, 2);
}

/**
 * \brief Sends the correct TOKEN on the line to start a write block transfer
 */
static void ROCODE sd_mmc_spi_start_write_block(void)
{
	uint8_t dummy = 0xFF;
	Assert(!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size));
	// Delay before start write block:
	// Nwr timing minimum = 8 cylces
	sendbytes(&dummy, 1);
	// Send start token
	uint8_t token;
	if (1 == sd_mmc_spi_nb_block) {
		token = SPI_TOKEN_SINGLE_WRITE;
	} else {
		token = SPI_TOKEN_MULTI_WRITE;
	}
	sendbytes(&token, 1);
}

/**
 * \brief Waits the TOKEN which notify the end of write block transfer
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_spi_err.
 */
static bool ROCODE sd_mmc_spi_stop_write_block(void)
{
	uint8_t resp = 0;
	uint16_t crc;

	// Send CRC
	crc = 0xFFFF; /// CRC is disabled in SPI mode
	sendbytes((uint8_t *)&crc, 2);
	// Receiv data response token
	readbytes(&resp, 1);
	if (!SPI_TOKEN_DATA_RESP_VALID(resp)) {
		sd_mmc_spi_err = SD_MMC_SPI_ERR;
		sd_mmc_spi_debug("%s: Invalid Data Response Token 0x%x\n\r", __func__, resp);
		return false;
	}
	// Check data response
	switch (SPI_TOKEN_DATA_RESP_CODE(resp)) {
	case SPI_TOKEN_DATA_RESP_ACCEPTED:
		break;
	case SPI_TOKEN_DATA_RESP_CRC_ERR:
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_CRC;
		sd_mmc_spi_debug("%s: Write blocks, SD_MMC_SPI_ERR_CRC, resp 0x%x\n\r",
				__func__, resp);
		return false;
	case SPI_TOKEN_DATA_RESP_WRITE_ERR:
	default:
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE;
		sd_mmc_spi_debug("%s: Write blocks SD_MMC_SPI_ERR_WR, resp 0x%x\n\r",
				__func__, resp);
		return false;
	}
	return true;
}

/**
 * \brief Executed the end of a multi blocks write transfer
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_spi_err.
 */
static bool ROCODE sd_mmc_spi_stop_multiwrite_block(void)
{
	uint8_t value;

	if (1 == sd_mmc_spi_nb_block) {
		return true; // Single block write
	}
	if (sd_mmc_spi_nb_block >
		(sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size)) {
		return true; // It is not the End of multi write
	}

	// Delay before start write block:
	// Nwr timing minimum = 8 cylces
	value = 0xFF;
	sendbytes(&value, 1);
	// Send stop token
	value = SPI_TOKEN_STOP_TRAN;
	sendbytes(&value, 1);
	// Wait busy
	if (!sd_mmc_spi_wait_busy()) {
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
		sd_mmc_spi_debug("%s: Stop write blocks timeout\n\r",
				__func__);
		return false;
	}
	return true;
}


//-------------------------------------------------------------------
//--------------------- PUBLIC FUNCTIONS ----------------------------

sd_mmc_spi_errno_t ROCODE sd_mmc_spi_get_errno(void)
{
	return sd_mmc_spi_err;
}

void ROCODE sd_mmc_spi_init()
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;

	//const uint32_t baudrate = SD_MMC_SPI_CLOCK / (2 * SDMMC_CLOCK_INIT) - 1;
  //spi_m_sync_set_baudrate(sd_mmc_master, baudrate);
  oc_set_sd_speed(0);

  gpio_set_pin_level(SD_CS, true);
}

void ROCODE sd_mmc_spi_select_device(uint32_t clock)
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;

#ifdef SD_MMC_SPI_MAX_CLOCK
	if (clock == 0 || clock > SD_MMC_SPI_MAX_CLOCK) {
		clock = SD_MMC_SPI_MAX_CLOCK;
	}
#endif
  oc_set_sd_speed(clock / 1000000);
#ifdef BITBANG_SPI
  gpio_set_pin_level(SD_MOSI, 1);
#else
  oc_spi_send_byte(255);
#endif
  gpio_set_pin_level(SD_CS, false);
}

void ROCODE sd_mmc_spi_deselect_device()
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
  gpio_set_pin_level(SD_CS, true);
}

void ROCODE sd_mmc_spi_send_clock(void)
{
	uint8_t i;
	uint8_t dummy = 0xFF;

	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	//! Send 80 cycles
	for (i = 0; i < 10; i++) {
		sendbytes(&dummy, 1); // 8 cycles
	}
}

bool ROCODE sd_mmc_spi_send_cmd(sdmmc_cmd_def_t cmd, uint32_t arg)
{
	return sd_mmc_spi_adtc_start(cmd, arg, 0, 0, false);
}

bool ROCODE sd_mmc_spi_adtc_start(sdmmc_cmd_def_t cmd, uint32_t arg,
		uint16_t block_size, uint16_t nb_block, bool access_block)
{
	uint8_t dummy = 0xFF;
	uint8_t cmd_token[6];
	uint8_t ncr_timeout;
	uint8_t r1; //! R1 response

	UNUSED(access_block);
	Assert(cmd & SDMMC_RESP_PRESENT); // Always a response in SPI mode
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;

	// Encode SPI command
	cmd_token[0] = SPI_CMD_ENCODE(SDMMC_CMD_GET_INDEX(cmd));
	cmd_token[1] = arg >> 24;
	cmd_token[2] = arg >> 16;
	cmd_token[3] = arg >> 8;
	cmd_token[4] = arg;
	cmd_token[5] = sd_mmc_spi_crc7(cmd_token, 5);

	// 8 cycles to respect Ncs timing
	// Note: This byte does not include start bit "0",
	// thus it is ignored by card.
	sendbytes(&dummy, 1);
	// Send command
	sendbytes(cmd_token, sizeof(cmd_token));

	// Wait for response
	// Two retry will be done to manage the Ncr timing between command and reponse
	// Ncr: Min. 1x8 clock  cycle, Max. 8x8 clock cycles
	// WORKAROUND for no compliance card (Atmel Internal ref. SD13):
	r1 = 0xFF;
	// Ignore first byte because Ncr min. = 8 clock cylces
	readbytes(&r1, 1);
	ncr_timeout = 7;
	while (1) {
		readbytes(&r1, 1); // 8 cycles
		if ((r1 & R1_SPI_ERROR) == 0) {
			// Valid R1 response
			break;
		}
		if (--ncr_timeout == 0) {
			// Here Valid R1 response received
			sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lX, R1 timeout\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg);
			sd_mmc_spi_err = SD_MMC_SPI_ERR_RESP_TIMEOUT;
			return false;
		}
	}

	// Save R1 (Specific to SPI interface) in 32 bit response
	// The R1_SPI_IDLE bit can be checked by high level
	sd_mmc_spi_response_32 = r1;

	// Manage error in R1
	if (r1 & R1_SPI_COM_CRC) {
		sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, r1 0x%02x, R1_SPI_COM_CRC\n\r",
				__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg, r1);
		sd_mmc_spi_err = SD_MMC_SPI_ERR_RESP_CRC;
		return false;
	}
	if (r1 & R1_SPI_ILLEGAL_COMMAND) {
		sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, r1 0x%x, R1 ILLEGAL_COMMAND\n\r",
				__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg, r1);
		sd_mmc_spi_err = SD_MMC_SPI_ERR_ILLEGAL_COMMAND;
		return false;
	}
	if (r1 & ~R1_SPI_IDLE) {
		// Other error
		sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, r1 0x%x, R1 error\n\r",
				__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg, r1);
		sd_mmc_spi_err = SD_MMC_SPI_ERR;
		return false;
	}

	// Manage other responses
	if (cmd & SDMMC_RESP_BUSY) {
		if (!sd_mmc_spi_wait_busy()) {
			sd_mmc_spi_err = SD_MMC_SPI_ERR_RESP_BUSY_TIMEOUT;
			sd_mmc_spi_debug("%s: cmd %02d, arg 0x%08lx, Busy signal always high\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), arg);
			return false;
		}
	}
	if (cmd & SDMMC_RESP_8) {
		sd_mmc_spi_response_32 = 0;
		readbytes((uint8_t *)&sd_mmc_spi_response_32, 1);
		sd_mmc_spi_response_32 = le32_to_cpu(sd_mmc_spi_response_32);
	}
	if (cmd & SDMMC_RESP_32) {
		readbytes((uint8_t *)&sd_mmc_spi_response_32, 4);
		sd_mmc_spi_response_32 = be32_to_cpu(sd_mmc_spi_response_32);
	}

	sd_mmc_spi_block_size = block_size;
	sd_mmc_spi_nb_block = nb_block;
	sd_mmc_spi_transfert_pos = 0;
	return true; // Command complete
}

uint32_t ROCODE sd_mmc_spi_get_response(void)
{
	return sd_mmc_spi_response_32;
}

bool ROCODE sd_mmc_spi_read_word(uint32_t* value)
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	Assert(sd_mmc_spi_nb_block >
			(sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size)) {
		// New block
		if (!sd_mmc_spi_start_read_block()) {
			return false;
		}
	}
	// Read data
	readbytes((uint8_t *)&value, 4);
	*value = le32_to_cpu(*value);
	sd_mmc_spi_transfert_pos += 4;

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size)) {
		// End of block
		sd_mmc_spi_stop_read_block();
	}
	return true;
}

bool ROCODE sd_mmc_spi_write_word(uint32_t value)
{
	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	Assert(sd_mmc_spi_nb_block >
			(sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size)) {
		// New block
		sd_mmc_spi_start_write_block();
	}

	// Write data
	value = cpu_to_le32(value);
	sendbytes((uint8_t*)&value, 4);
	sd_mmc_spi_transfert_pos += 4;

	if (!(sd_mmc_spi_transfert_pos % sd_mmc_spi_block_size)) {
		// End of block
		if (!sd_mmc_spi_stop_write_block()) {
			return false;
		}
		// Wait busy due to data programmation
		if (!sd_mmc_spi_wait_busy()) {
			sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
			sd_mmc_spi_debug("%s: Write blocks timeout\n\r", __func__);
			return false;
		}
	}
	return sd_mmc_spi_stop_multiwrite_block();
}

bool ROCODE sd_mmc_spi_start_read_blocks(void *dest, uint16_t nb_block)
{
	uint32_t pos;

	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	pos = 0;
	while (nb_block--) {
		Assert(sd_mmc_spi_nb_block >
				(sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));
		if (!sd_mmc_spi_start_read_block()) {
			return false;
		}

		// Read block
		readbytes(&((uint8_t*)dest)[pos],
			sd_mmc_spi_block_size);
		pos += sd_mmc_spi_block_size;
		sd_mmc_spi_transfert_pos += sd_mmc_spi_block_size;

		sd_mmc_spi_stop_read_block();
	}
	return true;
}

bool ROCODE sd_mmc_spi_wait_end_of_read_blocks(void)
{
	return true;
}

bool ROCODE sd_mmc_spi_start_write_blocks(const void *src, uint16_t nb_block)
{
	uint32_t pos;

	sd_mmc_spi_err = SD_MMC_SPI_NO_ERR;
	pos = 0;
	while (nb_block--) {
		Assert(sd_mmc_spi_nb_block >
				(sd_mmc_spi_transfert_pos / sd_mmc_spi_block_size));
		sd_mmc_spi_start_write_block();

		// Write block
		sendbytes(&((uint8_t*)src)[pos],
				sd_mmc_spi_block_size);
		pos += sd_mmc_spi_block_size;
		sd_mmc_spi_transfert_pos += sd_mmc_spi_block_size;

		if (!sd_mmc_spi_stop_write_block()) {
			return false;
		}
		// Do not check busy of last block
		// but delay it to mci_wait_end_of_write_blocks()
		if (nb_block) {
			// Wait busy due to data programmation
			if (!sd_mmc_spi_wait_busy()) {
				sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
				sd_mmc_spi_debug("%s: Write blocks timeout\n\r", __func__);
				return false;
			}
		}
	}
	return true;
}

bool ROCODE sd_mmc_spi_wait_end_of_write_blocks(void)
{
	// Wait busy due to data programmation of last block writed
	if (!sd_mmc_spi_wait_busy()) {
		sd_mmc_spi_err = SD_MMC_SPI_ERR_WRITE_TIMEOUT;
		sd_mmc_spi_debug("%s: Write blocks timeout\n\r", __func__);
		return false;
	}
	return sd_mmc_spi_stop_multiwrite_block();
}

//! @}

#endif // SD_MMC_SPI_MODE
