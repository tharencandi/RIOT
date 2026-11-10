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
 * @brief       OV2640 sensor registers and register table declarations
 *
 * Tables are defined in arducam_mini_ov2640.c and
 * arducam_mini_ov2640_readout.c; this header only declares them.
 * TODO: check license of upstream ArduCAM tables before upstreaming.
 *
 * @author      Tharen Emmanuel Candi <candi.tharen@gmail.com>
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B

#define OV2640_REG_BANK_SEL     0xFF    /* register bank selector */
#define OV2640_BANK_SENSOR      0x01    /* bank 1: sensor control + chip ID */
#define OV2640_REG_COM7         0x12    /* common control 7 (mode/reset) */
#define OV2640_COM7_RESET       0x80    /* COM7 bit 7: soft reset */
#define OV2640_REG_COM10        0x15    /* common control 10 */
#define OV2640_COM10_DEFAULT    0x00    /* clear all COM10 features */
#define OV2640_EXPECTED_VID     0x26    /* OmniVision manufacturer ID */
#define OV2640_EXPECTED_PID_1   0x41    /* OV2640 product ID */
#define OV2640_EXPECTED_PID_2   0x42    /* OV2640 product ID (rev 2) */

/* sensor bank orientation. bits 1:0 are AEC[1:0], the tables write 0 there */
#define OV2640_REG_REG04        0x04    /* [7] mirror, [6] flip, [4] VREF[0], [3] HREF[0] */
#define OV2640_REG04_BASE       0x28    /* bits 5 + 3 as in the arducam tables, no visible effect */
#define OV2640_REG04_HMIRROR    0x80    /* set = the right way round on this module */
#define OV2640_REG04_VFLIP      0x50    /* bit 6 + VREF[0]: bit 6 alone swaps the colours */

/* DSP bank (0xFF = 0x00) output scaling, written by the driver after a
   readout table (datasheet v1.6, table 12) */
#define OV2640_BANK_DSP         0x00
#define OV2640_REG_CTRLI        0x50    /* bit 7 LP_DP, [5:3] V_DIVIDER, [2:0] H_DIVIDER */
#define OV2640_CTRLI_LP_DP      0x80
#define OV2640_REG_HSIZE        0x51    /* image window width [7:0], real / 4 */
#define OV2640_REG_VSIZE        0x52    /* image window height [7:0], real / 4 */
#define OV2640_REG_XOFFL        0x53    /* window x offset [7:0], pixels */
#define OV2640_REG_YOFFL        0x54    /* window y offset [7:0], pixels */
#define OV2640_REG_VHYX         0x55    /* [7] V_SIZE[8], [6:4] Y_OFF[10:8], [3] H_SIZE[8], [2:0] X_OFF[10:8] */
#define OV2640_REG_TEST         0x57    /* [7] H_SIZE[9] */
#define OV2640_REG_ZMOW         0x5A    /* OUTW[7:0], real / 4 */
#define OV2640_REG_ZMOH         0x5B    /* OUTH[7:0], real / 4 */
#define OV2640_REG_ZMHH         0x5C    /* [2] OUTH[8], [1:0] OUTW[9:8] */
#define OV2640_REG_RESET        0xE0    /* DSP block resets, 0x00 = run */
#define OV2640_RESET_DVP        0x04    /* RESET: hold the DVP (output) in reset */
#define OV2640_REG_CTRL2        0x86    /* DSP module enables */
#define OV2640_REG_R_DVP_SP     0xD3    /* DVP pixel clock divider */

/**
 * @brief   One register write: (0xFF, 0xFF) ends a table
 */
struct sensor_reg {
    uint16_t reg;   /**< register address */
    uint16_t val;   /**< value to write */
};

/* format base tables (arducam_mini_ov2640.c) */
extern const struct sensor_reg OV2640_RGB565_BASE[];  /* RGB565, UXGA readout */
extern const struct sensor_reg OV2640_JPEG_INIT[];    /* JPEG, then YUV422 + JPEG */
extern const struct sensor_reg OV2640_YUV422[];
extern const struct sensor_reg OV2640_JPEG[];

/* readout-mode switch tables (arducam_mini_ov2640_readout.c), applied
   after a base table; they end in the DSP bank with the DVP in reset */
extern const struct sensor_reg OV2640_READOUT_UXGA[];
extern const struct sensor_reg OV2640_READOUT_SVGA[];
extern const struct sensor_reg OV2640_READOUT_CIF[];


#ifdef __cplusplus
}
#endif

/** @} */
