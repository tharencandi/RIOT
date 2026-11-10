/*
 * SPDX-FileCopyrightText: 2026 Tharen Emmanuel Candi <candi.tharen@gmail.com>
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#pragma once

/**
 * @defgroup    drivers_arducam_mini ArduCAM Mini 2MP Plus SPI camera
 * @ingroup     drivers_multimedia
 * @brief       Driver for the ArduCAM Mini 2MP Plus (OV2640 sensor + ArduChip FIFO)
 *
 * ## Description
 *
 * @todo describe the module once the API settles:
 * - ArduChip (CPLD) with a frame FIFO on SPI, OV2640 sensor on I2C (SCCB)
 * - output modes: JPEG and raw RGB565 at the tested sizes, each with the
 *   sensor readouts that work for it (see @ref arducam_mini_mode_t),
 *   changeable at runtime
 * - sensor readout modes (see @ref arducam_mini_readout_t) and field of view
 *   (see @ref arducam_mini_fov_t), picked by the application
 * - start / wait capture, so the camera captures while the CPU works
 *
 * ## Usage
 *
 * ```c
 * // static: frames are too big for a thread stack
 * static uint8_t frame[ARDUCAM_MINI_RGB565_SIZE(80, 60)];
 * arducam_mini_t cam;
 *
 * arducam_mini_init(&cam, &arducam_mini_params[0]);
 * int32_t len = arducam_mini_capture(&cam, frame, sizeof(frame), 1000);
 * if (len > 0) {
 *     // frame[0 .. len - 1] holds one frame
 * }
 * ```
 *
 * ## Thread safety
 *
 * @todo document the contract:
 * - use one device from one thread only
 * - never call from interrupt context (asserted)
 *
 * ## References
 *
 * @todo links: ArduCAM Mini 2MP hardware application note, OV2640 datasheet,
 *       ArduCAM Arduino library (register tables), RIOT-OS/RIOT#21057
 *
 * @{
 *
 * @file
 * @brief       Public API of the ArduCAM Mini 2MP Plus driver
 *
 * @author      Tharen Emmanuel Candi <candi.tharen@gmail.com>
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "periph/i2c.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Output format
 */
typedef enum {
    ARDUCAM_MINI_FMT_RGB565,    /**< raw RGB565, 2 bytes per pixel, big endian */
    ARDUCAM_MINI_FMT_JPEG,      /**< JPEG, variable size */
    /* later: ARDUCAM_MINI_FMT_Y8 (greyscale, 1 byte per pixel) */
} arducam_mini_format_t;

/**
 * @brief   Sensor readout mode
 *
 * How the OV2640 reads its pixel array. All three cover the full field of
 * view; SVGA and CIF sub-sample it in the sensor, so a frame takes less
 * time, and the DSP scales the smaller image down to the output size.
 * Speed against picture quality is a real trade-off, so the application
 * picks it (there is no AUTO):
 *
 * | readout | DSP input | frame time raw | frame time JPEG | picture            |
 * |---------|-----------|----------------|-----------------|--------------------|
 * | SVGA    | 800x600   | 35.0 ms        | 36.4 ms         | good (default)     |
 * | UXGA    | 1600x1200 | 68.4 ms        | 140 ms          | best               |
 * | CIF     | 400x296   | 18.3 ms        | -               | fastest, less accurate colour |
 *
 * Measured on one OV2640 (PID 0x42) in bright light. JPEG at UXGA runs at
 * half the sensor clock (CLKRC /2, from ArduCAM's tables). Which readouts
 * work for which size: see @ref arducam_mini_mode_t.
 */
typedef enum {
    ARDUCAM_MINI_READOUT_SVGA = 0,  /**< 800x600, ~28 fps (default) */
    ARDUCAM_MINI_READOUT_UXGA,      /**< 1600x1200, ~14 fps raw, ~7 fps JPEG */
    ARDUCAM_MINI_READOUT_CIF,       /**< 400x296, ~55 fps, RGB565 only */
} arducam_mini_readout_t;

/**
 * @brief   Field of view
 *
 * FULL keeps the whole sensor area. CROP takes a centred window first and
 * scales that, so tiny sizes keep more detail per pixel. The share of the
 * view a crop keeps is listed per size in @ref arducam_mini_mode_t; it is
 * the same in every readout, so switching readout for speed doesnt change
 * what the camera sees.
 */
typedef enum {
    ARDUCAM_MINI_FOV_FULL = 0,      /**< whole sensor area (default) */
    ARDUCAM_MINI_FOV_CROP,          /**< centred crop, sharper at tiny sizes */
} arducam_mini_fov_t;

/**
 * @brief   Everything that describes a frame
 *
 * Only these combinations are supported, arducam_mini_init() and
 * arducam_mini_set_mode() return -EINVAL for anything else. Every one passed
 * the conformance bench in tests/drivers/arducam_mini (exact frame size or a
 * complete JPEG with the right size, 5 x 3 frames, picture checked):
 *
 * | format | size                                      | fov         | readouts        |
 * |--------|-------------------------------------------|-------------|-----------------|
 * | RGB565 | 40x32, 48x36                              | CROP (60 %) | SVGA, CIF       |
 * | RGB565 | 80x60                                     | FULL        | SVGA, CIF       |
 * | RGB565 | 160x120, 176x144                          | FULL        | SVGA, UXGA, CIF |
 * | RGB565 | 320x240, 352x288                          | FULL        | SVGA, UXGA      |
 * | JPEG   | 160x120, 176x144, 320x240, 352x288, 640x480, 800x600 | FULL | SVGA, UXGA |
 * | JPEG   | 1024x768, 1280x1024, 1600x1200            | FULL        | UXGA            |
 *
 * Why not more: the DSP breaks below a zoom of 0.5 (an extra line, and the
 * last pixels of the picture wrong), when the width after its pre-divider
 * isnt a multiple of 4, and CIF without the divider shows line artefacts.
 * Details and the measurements: tharencandi/riot-arducam-mini issue #2.
 *
 * @note    Observed on one OV2640 (PID 0x42). On the PID 0x41 samples we
 *          have, sensor settings dont take effect (no readout switching),
 *          still being investigated.
 * @todo    replicate on a second 0x42 sample
 */
typedef struct {
    arducam_mini_format_t format;   /**< output format */
    uint16_t width;                 /**< output width in pixels */
    uint16_t height;                /**< output height in pixels */
    arducam_mini_readout_t readout; /**< sensor readout, SVGA if unsure */
    arducam_mini_fov_t fov;         /**< field of view, usually FULL */
} arducam_mini_mode_t;

/**
 * @brief   Frame size in bytes of a raw RGB565 frame, for static buffers
 */
#define ARDUCAM_MINI_RGB565_SIZE(w, h)  ((size_t)(w) * (size_t)(h) * 2U)

/**
 * @brief   Exact frame size in bytes for a mode
 *
 * @param[in]   mode    output mode
 *
 * @return  width * height * 2 for RGB565, 0 for JPEG (variable size,
 *          bounded at capture time by the FIFO length register)
 */
static inline size_t arducam_mini_frame_size(const arducam_mini_mode_t *mode)
{
    return (mode->format == ARDUCAM_MINI_FMT_RGB565)
           ? ARDUCAM_MINI_RGB565_SIZE(mode->width, mode->height) : 0;
}

/**
 * @brief   Device initialization parameters (bus wiring + initial mode)
 */
typedef struct {
    spi_t spi;                      /**< SPI bus of the ArduChip */
    spi_cs_t spi_cs;                /**< SPI chip select */
    spi_clk_t spi_clk;              /**< SPI clock, max 8 MHz */
    i2c_t i2c;                      /**< I2C bus of the OV2640 sensor */
    uint8_t i2c_addr;               /**< 7-bit sensor address (0x30) */
    bool hflip;                     /**< mirror the picture left-right, e.g. for how
                                         the camera is mounted (false = as seen) */
    bool vflip;                     /**< flip the picture upside down */
    arducam_mini_mode_t mode;       /**< mode applied at init */
} arducam_mini_params_t;

/**
 * @brief   Capture state, so wait knows if start was called
 */
typedef enum {
    ARDUCAM_MINI_IDLE,      /**< no capture running */
    ARDUCAM_MINI_CAPTURING, /**< start called, wait not done yet */
} arducam_mini_state_t;

/**
 * @brief   Device descriptor
 */
typedef struct {
    arducam_mini_params_t params;   /**< device parameters (copied at init) */
    arducam_mini_state_t state;     /**< idle or capturing */
    arducam_mini_mode_t mode;       /**< current mode */
} arducam_mini_t;

/**
 * @brief   Initialize the camera and configure it per @p params
 *
 * Resets the ArduChip, checks it over SPI, probes the OV2640 over I2C
 * (up to @ref CONFIG_ARDUCAM_MINI_PROBE_RETRIES times) and applies
 * params->mode like arducam_mini_set_mode().
 *
 * @param[out]  dev     device descriptor to initialize
 * @param[in]   params  initialization parameters
 *
 * @retval  0           on success
 * @retval  -ENXIO      chip select could not be initialized
 * @retval  -ENODEV     no ArduChip on SPI or no OV2640 on I2C (wrong chip ID)
 * @retval  -EIO        I2C communication with the sensor failed
 * @retval  -EINVAL     params->mode is not a supported combination (see @ref arducam_mini_mode_t)
 */
int arducam_mini_init(arducam_mini_t *dev, const arducam_mini_params_t *params);

/**
 * @brief   Change the output mode (format, size, readout, field of view)
 *
 * Rewrites the sensor registers and waits for them to settle (~300 ms,
 * datasheet). Only allowed while no capture is running.
 *
 * @param[in,out]   dev     device descriptor
 * @param[in]       mode    new mode
 *
 * @retval  0           on success
 * @retval  -EBUSY      a capture is running
 * @retval  -EINVAL     @p mode is not a supported combination (see @ref arducam_mini_mode_t)
 * @retval  -EIO        I2C communication with the sensor failed
 */
int arducam_mini_set_mode(arducam_mini_t *dev, const arducam_mini_mode_t *mode);

/**
 * @brief   Start a capture and return immediately
 *
 * The frame is written into the ArduChip FIFO in the background. Collect
 * it with arducam_mini_capture_wait().
 *
 * @param[in,out]   dev     device descriptor
 *
 * @retval  0           capture started
 * @retval  -EBUSY      a capture is already running
 */
int arducam_mini_capture_start(arducam_mini_t *dev);

/**
 * @brief   Wait for the capture started by arducam_mini_capture_start()
 *          and read the frame into @p buf
 *
 * After any return value except -EINVAL the device is idle and the FIFO
 * is cleared, so the next capture can be started.
 *
 * @todo document: the timeout starts when wait is called, so a frame that
 *       finished during other work returns without polling
 *
 * @param[in,out]   dev         device descriptor
 * @param[out]      buf         buffer for the frame
 * @param[in]       max_len     size of @p buf in bytes
 * @param[in]       timeout_ms  maximum time to wait for the capture
 *
 * @return  frame length in bytes (> 0) on success
 * @retval  -EINVAL     no capture was started
 * @retval  -ETIMEDOUT  the capture did not finish within @p timeout_ms
 * @retval  -ENOBUFS    @p buf is too small for the frame
 * @retval  -EIO        the FIFO length doesnt match the raw frame (plus up to
 *                      16 pad bytes), or no complete JPEG (ff d8 ... ff d9)
 */
int32_t arducam_mini_capture_wait(arducam_mini_t *dev, uint8_t *buf,
                                  size_t max_len, uint32_t timeout_ms);

/**
 * @brief   Blocking capture: arducam_mini_capture_start() followed by
 *          arducam_mini_capture_wait()
 *
 * Returns a length or a negative errno, so the type is int32_t, not
 * size_t: frames can be up to 8 MB and int is only 16 bit on AVR.
 *
 * @param[in,out]   dev         device descriptor
 * @param[out]      buf         buffer for the frame
 * @param[in]       max_len     size of @p buf in bytes
 * @param[in]       timeout_ms  maximum time to wait for the capture
 *
 * @return  frame length in bytes (> 0) on success
 * @retval  -EBUSY      a capture is already running
 * @retval  -ETIMEDOUT  the capture did not finish within @p timeout_ms
 * @retval  -ENOBUFS    @p buf is too small for the frame
 * @retval  -EIO        the FIFO length doesnt match the raw frame (plus up to
 *                      16 pad bytes), or no complete JPEG (ff d8 ... ff d9)
 */
int32_t arducam_mini_capture(arducam_mini_t *dev, uint8_t *buf,
                             size_t max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

/** @} */
