/*
 * SPDX-FileCopyrightText: 2011-2015 ArduCAM.com
 * SPDX-FileCopyrightText: 2026 Tharen Emmanuel Candi <candi.tharen@gmail.com>
 * SPDX-License-Identifier: LGPL-2.1-only
 */

/**
 * @ingroup     drivers_arducam_mini
 * @{
 *
 * @file
 * @brief       OV2640 readout-mode switch tables (UXGA, SVGA, CIF)
 *
 * Switch the sensor readout from UXGA (set by the raw RGB565 init table)
 * to SVGA (800 x 600) or CIF (400 x 296). Both cover the whole pixel
 * array (sub-sampled), so the field of view stays the same; the driver
 * then programs the DSP scaler for the output size and releases the DVP
 * reset. Usage contract and derivation: notes/ov2640_readout_modes.md.
 *
 * Provenance (full trace in the notes file, section "Provenance"):
 *   [A1] ArduCAM ov2640_regs.h (github.com/ArduCAM/Arduino @ 066a7ea1),
 *        sensor block shared by the 160x120..352x288 JPEG size tables,
 *        i.e. the SVGA readout this board already runs in the JPEG path
 *   [A2] ArduCAM OV2640_JPEG_INIT, final value in that working SVGA state
 *   [D]  OV2640 datasheet v1.6 (facts only: addresses, bit fields,
 *        per-mode defaults, frame timing)
 *   [C]  computed from [D]; arithmetic in the notes file
 *   [X]  not covered by a board test yet
 * No values were taken from esp32-camera (Apache-2.0), Linux ov2640.c
 * (GPL-2.0-only) or Adafruit_CircuitPython_OV2640 (port of esp32-camera).
 *
 * Change notice (LGPL-2.1 section 2(b)): created 2026-10 by Tharen
 * Emmanuel Candi from the ArduCAM tables above: sensor block of the
 * 160x120 JPEG size table plus four JPEG_INIT registers, COM1 added,
 * DSP part reduced to the input size, CIF variant derived from [D].
 * UXGA table added 2026-10-12: sensor block + DSP input size of the
 * 640x480..1600x1200 JPEG size tables, scaler part removed (the driver
 * calculates it).
 *
 * @author      Tharen Emmanuel Candi <candi.tharen@gmail.com>
 */

#include "arducam_mini_ov2640_regs.h"

const struct sensor_reg OV2640_READOUT_SVGA[] = {
    { 0xff, 0x01 },     /* sensor bank                                      */

    { 0x12, 0x40 },     /* COM7    SVGA readout (6:4 = 100)          [A1][D] */
    { 0x03, 0x0a },     /* COM1    VSTRT/VEND LSBs 2/2 (SVGA default)    [D] */

    /* readout window 800 x 600 */
    { 0x17, 0x11 },     /* HREFST  137                               [A1][D] */
    { 0x18, 0x43 },     /* HREFEND 537 -> 400 units = 800 px         [A1][D] */
    { 0x19, 0x00 },     /* VSTRT   2                                 [A1][D] */
    { 0x1a, 0x4b },     /* VEND    302 -> 300 units = 600 lines      [A1][C] */
    { 0x32, 0x09 },     /* REG32   window LSBs, PCLK not divided     [A1][D] */

    /* banding steps for tLINE = 49.6 us */
    { 0x4f, 0xca },     /* BD50    10 ms / 49.6 us = 202             [A1][C] */
    { 0x50, 0xa8 },     /* BD60    8.33 ms / 49.6 us = 168           [A1][C] */

    /* reserved / partly documented, mode dependent */
    { 0x5a, 0x23 },     /*                                              [A1] */
    { 0x6d, 0x00 },     /*                                              [A1] */
    { 0x39, 0x12 },     /*                                              [A1] */
    { 0x35, 0xda },     /*                                              [A1] */
    { 0x22, 0x1a },     /*                                              [A1] */
    { 0x37, 0xc3 },     /*                                              [A1] */
    { 0x23, 0x00 },     /*                                              [A1] */
    { 0x34, 0xc0 },     /* ARCOM2                                       [A1] */
    { 0x36, 0x1a },     /*                                              [A1] */
    { 0x06, 0x88 },     /*                                              [A1] */
    { 0x07, 0xc0 },     /*                                              [A1] */
    { 0x0d, 0x87 },     /* COM4                                         [A1] */
    { 0x0e, 0x41 },     /*                                              [A1] */
    { 0x4c, 0x00 },     /*                                              [A1] */
    { 0x3d, 0x38 },     /* (raw UXGA: 0x34)                             [A2] */
    { 0x42, 0x03 },     /*                                              [A2] */
    { 0x48, 0x00 },     /* COM19   zoom v-start LSBs                 [A2][D] */
    { 0x5b, 0x00 },     /*                                              [A2] */

    /* DSP input size 800 x 600; DVP held in reset, the driver releases it */
    { 0xff, 0x00 },     /* DSP bank                                         */
    { 0xe0, 0x04 },     /* RESET   DVP                               [A1][D] */
    { 0xc0, 0x64 },     /* HSIZE8  800 / 8                           [A1][C] */
    { 0xc1, 0x4b },     /* VSIZE8  600 / 8                           [A1][C] */
    { 0x8c, 0x00 },     /* SIZEL                                     [A2][C] */
    { 0x51, 0xc8 },     /* HSIZE   800 / 4                           [A1][C] */
    { 0x52, 0x96 },     /* VSIZE   600 / 4                           [A1][C] */
    { 0x53, 0x00 },     /* XOFFL                                     [A1][C] */
    { 0x54, 0x00 },     /* YOFFL                                     [A1][C] */
    { 0x55, 0x00 },     /* VHYX    size/offset high bits             [A1][C] */
    { 0x57, 0x00 },     /* TEST    H_SIZE[9]                         [A1][C] */

    { 0xff, 0xff },
};

/* same register set and order as SVGA; differences marked CIF */
const struct sensor_reg OV2640_READOUT_CIF[] = {
    { 0xff, 0x01 },

    { 0x12, 0x20 },     /* COM7    CIF readout (6:4 = 010; v1.6 datasheet
                         *         says 001, board confirms 010)    CIF [D] */
    { 0x03, 0x0a },     /* COM1    VSTRT/VEND LSBs 2/2              CIF [C] */

    /* readout window: horizontal = datasheet CIF defaults, 296 lines */
    { 0x17, 0x11 },     /* HREFST                                       [D] */
    { 0x18, 0x43 },     /* HREFEND                                      [D] */
    { 0x19, 0x00 },     /* VSTRT   2                                    [D] */
    { 0x1a, 0x25 },     /* VEND    150 -> 148 units = 296 lines    CIF [C] */
    { 0x32, 0x89 },     /* REG32   window LSBs, PCLK / 2 -> 60 fps CIF [D][C] */

    /* banding: CIF tLINE at 60 fps = SVGA tLINE at 30 fps */
    { 0x4f, 0xca },     /* BD50                                         [C] */
    { 0x50, 0xa8 },     /* BD60                                         [C] */

    /* reserved: unchanged from SVGA (same line readout, fewer lines) */
    { 0x5a, 0x23 },     /*                                          [A1][X] */
    { 0x6d, 0x00 },     /*                                          [A1][X] */
    { 0x39, 0x12 },     /*                                          [A1][X] */
    { 0x35, 0xda },     /*                                          [A1][X] */
    { 0x22, 0x1a },     /*                                          [A1][X] */
    { 0x37, 0xc3 },     /*                                          [A1][X] */
    { 0x23, 0x00 },     /*                                          [A1][X] */
    { 0x34, 0xc0 },     /*                                          [A1][X] */
    { 0x36, 0x1a },     /*                                          [A1][X] */
    { 0x06, 0x88 },     /*                                          [A1][X] */
    { 0x07, 0xc0 },     /*                                          [A1][X] */
    { 0x0d, 0x87 },     /*                                          [A1][X] */
    { 0x0e, 0x41 },     /*                                          [A1][X] */
    { 0x4c, 0x00 },     /*                                          [A1][X] */
    { 0x3d, 0x38 },     /*                                          [A2][X] */
    { 0x42, 0x03 },     /*                                          [A2][X] */
    { 0x48, 0x00 },     /*                                          [A2][D] */
    { 0x5b, 0x00 },     /*                                          [A2][X] */

    /* DSP input size 400 x 296 */
    { 0xff, 0x00 },
    { 0xe0, 0x04 },     /* RESET   DVP                                  [D] */
    { 0xc0, 0x32 },     /* HSIZE8  400 / 8                          CIF [C] */
    { 0xc1, 0x25 },     /* VSIZE8  296 / 8                          CIF [C] */
    { 0x8c, 0x00 },     /* SIZEL                                        [C] */
    { 0x51, 0x64 },     /* HSIZE   400 / 4                          CIF [C] */
    { 0x52, 0x4a },     /* VSIZE   296 / 4                          CIF [C] */
    { 0x53, 0x00 },     /* XOFFL                                        [C] */
    { 0x54, 0x00 },     /* YOFFL                                        [C] */
    { 0x55, 0x00 },     /* VHYX                                         [C] */
    { 0x57, 0x00 },     /* TEST                                         [C] */

    { 0xff, 0xff },
};

/* UXGA readout for the JPEG path. JPEG_INIT leaves the sensor in SVGA, so
 * JPEG sizes that need UXGA apply this. Sensor block shared by the ArduCAM
 * 640x480..1600x1200 JPEG size tables (identical in all five, same order),
 * DSP input 1600 x 1200. The raw path does not need it: OV2640_RGB565_BASE
 * already runs UXGA. */
const struct sensor_reg OV2640_READOUT_UXGA[] = {
    { 0xff, 0x01 },     /* sensor bank                                      */

    { 0x11, 0x01 },     /* CLKRC   clock / 2                         [A1][D] */
    { 0x12, 0x00 },     /* COM7    UXGA readout (6:4 = 000)          [A1][D] */

    /* readout window 1600 x 1200 */
    { 0x17, 0x11 },     /* HREFST                                    [A1][D] */
    { 0x18, 0x75 },     /* HREFEND                                   [A1][D] */
    { 0x32, 0x36 },     /* REG32   window LSBs                       [A1][D] */
    { 0x19, 0x01 },     /* VSTRT                                     [A1][D] */
    { 0x1a, 0x97 },     /* VEND                                      [A1][D] */
    { 0x03, 0x0f },     /* COM1    VSTRT/VEND LSBs                   [A1][D] */
    { 0x37, 0x40 },     /*                                              [A1] */

    /* banding steps */
    { 0x4f, 0xbb },     /* BD50                                      [A1][D] */
    { 0x50, 0x9c },     /* BD60                                      [A1][D] */

    /* reserved / partly documented, mode dependent */
    { 0x5a, 0x57 },     /*                                              [A1] */
    { 0x6d, 0x80 },     /*                                              [A1] */
    { 0x3d, 0x34 },     /*                                              [A1] */
    { 0x39, 0x02 },     /*                                              [A1] */
    { 0x35, 0x88 },     /*                                              [A1] */
    { 0x22, 0x0a },     /*                                              [A1] */
    { 0x37, 0x40 },     /* (written twice in the ArduCAM tables)        [A1] */
    { 0x34, 0xa0 },     /* ARCOM2                                       [A1] */
    { 0x06, 0x02 },     /*                                              [A1] */
    { 0x0d, 0xb7 },     /* COM4                                         [A1] */
    { 0x0e, 0x01 },     /*                                              [A1] */

    /* DSP input size 1600 x 1200; DVP held in reset, the driver releases it */
    { 0xff, 0x00 },     /* DSP bank                                         */
    { 0xe0, 0x04 },     /* RESET   DVP                               [A1][D] */
    { 0xc0, 0xc8 },     /* HSIZE8  1600 / 8                          [A1][C] */
    { 0xc1, 0x96 },     /* VSIZE8  1200 / 8                          [A1][C] */
    { 0x8c, 0x00 },     /* SIZEL                                     [A1][C] */
    { 0x51, 0x90 },     /* HSIZE   1600 / 4 = 0x190, low byte        [A1][C] */
    { 0x52, 0x2c },     /* VSIZE   1200 / 4 = 0x12c, low byte        [A1][C] */
    { 0x53, 0x00 },     /* XOFFL                                     [A1][C] */
    { 0x54, 0x00 },     /* YOFFL                                     [A1][C] */
    { 0x55, 0x88 },     /* VHYX    HSIZE[8] + VSIZE[8]               [A1][C] */
    { 0x57, 0x00 },     /* TEST    H_SIZE[9]                         [A1][C] */

    { 0xff, 0xff },
};

/*
 * MIT notice of the ArduCAM/Arduino repository root, reproduced in case
 * that licence (rather than the LGPL-2.1 of the library folder) applies
 * to the [A1]/[A2] values:
 *
 *   The MIT License (MIT)
 *
 *   Copyright (c) 2015 Lee
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a
 *   copy of this software and associated documentation files (the
 *   "Software"), to deal in the Software without restriction, including
 *   without limitation the rights to use, copy, modify, merge, publish,
 *   distribute, sublicense, and/or sell copies of the Software, and to
 *   permit persons to whom the Software is furnished to do so, subject to
 *   the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included
 *   in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/** @} */
