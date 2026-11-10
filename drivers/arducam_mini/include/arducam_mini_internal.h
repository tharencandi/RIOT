/*
 * SPDX-FileCopyrightText: 2026 Tharen Emmanuel Candi <candi.tharen@gmail.com>
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#pragma once

/**
 * @ingroup     drivers_arducam_mini
 * @{
 *
 * @file
 * @brief       ArduChip register map, FIFO commands, masks and timing
 *
 * credit: reference (RIOT-OS/RIOT#21057), which was based off the ArduCAM drivers
 *
 * @author      Tharen Emmanuel Candi <candi.tharen@gmail.com>
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    ArduChip registers and commands
 * @{
 */
#define ARDUCAM_MINI_REG_TEST1        0x00   /**< test register */
#define ARDUCAM_MINI_REG_FIFO         0x04   /**< FIFO control */
#define ARDUCAM_MINI_REG_GPIO_WRITE   0x06   /**< sensor reset / pwdn / pwren pins, as written */
#define ARDUCAM_MINI_REG_CPLD_RESET   0x07   /**< CPLD reset */
#define ARDUCAM_MINI_REG_VERSION      0x40   /**< ArduChip version */
#define ARDUCAM_MINI_REG_TRIG         0x41   /**< trigger / status */
#define ARDUCAM_MINI_REG_FIFO_SIZE1   0x42   /**< FIFO length [7:0] */
#define ARDUCAM_MINI_REG_FIFO_SIZE2   0x43   /**< FIFO length [15:8] */
#define ARDUCAM_MINI_REG_FIFO_SIZE3   0x44   /**< FIFO length [22:16] */
#define ARDUCAM_MINI_REG_GPIO_READ    0x45   /**< sensor reset / pwdn / pwren pin levels */
#define ARDUCAM_MINI_CMD_BURST_READ   0x3C   /**< burst FIFO read command */
/** @} */

/**
 * @name    ArduChip bits and masks
 * @{
 */
#define ARDUCAM_MINI_WRITE_BIT        0x80   /**< OR into reg addr for writes */
#define ARDUCAM_MINI_CPLD_RESET_MASK  0x80   /**< value written to CPLD_RESET to assert reset */
#define ARDUCAM_MINI_FIFO_CLEAR_MASK  0x01   /**< clear the FIFO write done flag */
#define ARDUCAM_MINI_FIFO_START_MASK  0x02   /**< start a capture */
#define ARDUCAM_MINI_CAP_DONE_MASK    0x08   /**< TRIG: capture done */
#define ARDUCAM_MINI_FIFO_MAX_SIZE    0x7fffff   /**< largest valid FIFO length */
#define ARDUCAM_MINI_FIFO_PAD_MAX     16     /**< most pad bytes after a raw frame (UC-474 +8, UC-C78 +16) */
/** @} */

/**
 * @name    JPEG frame markers (ISO/IEC 10918-1)
 * @{
 */
#define ARDUCAM_MINI_JPEG_MARKER_PREFIX 0xFF   /**< marker prefix */
#define ARDUCAM_MINI_JPEG_SOI           0xD8   /**< Start Of Image */
#define ARDUCAM_MINI_JPEG_EOI           0xD9   /**< End Of Image */
/** @} */

/**
 * @name    Timing
 * @{
 */
#define ARDUCAM_MINI_RESET_DELAY_MS     100    /**< CPLD and sensor reset recovery */
#define ARDUCAM_MINI_SETTLE_DELAY_MS    300    /**< OV2640 register setting settle time (datasheet) */
#define ARDUCAM_MINI_CS_SETTLE_MS       10     /**< CS high time to resync the ArduChip SPI logic */
#define ARDUCAM_MINI_SPI_TEST_RETRIES   3      /**< ArduChip reset + SPI test attempts */
/** @} */

#ifdef __cplusplus
}
#endif

/** @} */
