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
 * @brief       Default configuration for the ArduCAM Mini 2MP Plus driver
 *
 * Every value is a guarded default, so the application can override it at
 * build time (e.g. a different SPI bus or chip select on another board).
 *
 * @author      Tharen Emmanuel Candi <candi.tharen@gmail.com>
 */

#include "board.h"
#include "arducam_mini.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   How many times init probes the OV2640 before giving up
 *
 * Same pattern as si70xx / amg88xx: 3 tries, wait only after a failed one.
 * Set it higher while debugging the hardware.
 */
#ifndef CONFIG_ARDUCAM_MINI_PROBE_RETRIES
#  define CONFIG_ARDUCAM_MINI_PROBE_RETRIES     (3U)
#endif

/**
 * @brief   Poll interval in ms while waiting for a capture to finish
 *
 * The thread sleeps between polls, so other threads (inference) get the CPU.
 * A capture takes ~90-120 ms, so 1 ms polling costs almost nothing.
 */
#ifndef CONFIG_ARDUCAM_MINI_POLL_MS
#  define CONFIG_ARDUCAM_MINI_POLL_MS           (1U)
#endif

/**
 * @name    Default configuration parameters for the ArduCAM Mini driver
 * @{
 */
#ifndef ARDUCAM_MINI_PARAM_SPI
#  define ARDUCAM_MINI_PARAM_SPI        SPI_DEV(0)                      /**< SPI bus */
#endif
#ifndef ARDUCAM_MINI_PARAM_SPI_CS
#  define ARDUCAM_MINI_PARAM_SPI_CS     SPI_HWCS(0)                     /**< SPI chip select */
#endif
#ifndef ARDUCAM_MINI_PARAM_SPI_CLK
#  define ARDUCAM_MINI_PARAM_SPI_CLK    SPI_CLK_5MHZ                    /**< SPI clock (max 8 MHz) */
#endif
#ifndef ARDUCAM_MINI_PARAM_I2C
#  define ARDUCAM_MINI_PARAM_I2C        I2C_DEV(0)                      /**< I2C bus */
#endif
#ifndef ARDUCAM_MINI_PARAM_I2C_ADDR
#  define ARDUCAM_MINI_PARAM_I2C_ADDR   (0x30)                          /**< OV2640 address */
#endif
#ifndef ARDUCAM_MINI_PARAM_HFLIP
#  define ARDUCAM_MINI_PARAM_HFLIP      (false)                         /**< mirror left-right */
#endif
#ifndef ARDUCAM_MINI_PARAM_VFLIP
#  define ARDUCAM_MINI_PARAM_VFLIP      (false)                         /**< flip upside down */
#endif
#ifndef ARDUCAM_MINI_PARAM_FORMAT
#  define ARDUCAM_MINI_PARAM_FORMAT     ARDUCAM_MINI_FMT_JPEG           /**< output format */
#endif
#ifndef ARDUCAM_MINI_PARAM_WIDTH
#  define ARDUCAM_MINI_PARAM_WIDTH      (320)                           /**< output width */
#endif
#ifndef ARDUCAM_MINI_PARAM_HEIGHT
#  define ARDUCAM_MINI_PARAM_HEIGHT     (240)                           /**< output height */
#endif
#ifndef ARDUCAM_MINI_PARAM_READOUT
#  define ARDUCAM_MINI_PARAM_READOUT    ARDUCAM_MINI_READOUT_SVGA       /**< sensor readout */
#endif
#ifndef ARDUCAM_MINI_PARAM_FOV
#  define ARDUCAM_MINI_PARAM_FOV        ARDUCAM_MINI_FOV_FULL           /**< field of view */
#endif

/**
 * @brief   Default parameter set
 */
#ifndef ARDUCAM_MINI_PARAMS
#  define ARDUCAM_MINI_PARAMS           { .spi      = ARDUCAM_MINI_PARAM_SPI,      \
                                          .spi_cs   = ARDUCAM_MINI_PARAM_SPI_CS,   \
                                          .spi_clk  = ARDUCAM_MINI_PARAM_SPI_CLK,  \
                                          .i2c      = ARDUCAM_MINI_PARAM_I2C,      \
                                          .i2c_addr = ARDUCAM_MINI_PARAM_I2C_ADDR, \
                                          .hflip    = ARDUCAM_MINI_PARAM_HFLIP,    \
                                          .vflip    = ARDUCAM_MINI_PARAM_VFLIP,    \
                                          .mode     = {                            \
                                              .format  = ARDUCAM_MINI_PARAM_FORMAT,  \
                                              .width   = ARDUCAM_MINI_PARAM_WIDTH,   \
                                              .height  = ARDUCAM_MINI_PARAM_HEIGHT,  \
                                              .readout = ARDUCAM_MINI_PARAM_READOUT, \
                                              .fov     = ARDUCAM_MINI_PARAM_FOV,     \
                                          } }
#endif
/** @} */

/**
 * @brief   Configuration for each camera (one entry per device)
 */
static const arducam_mini_params_t arducam_mini_params[] =
{
    ARDUCAM_MINI_PARAMS
};

#ifdef __cplusplus
}
#endif

/** @} */
