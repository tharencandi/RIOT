/*
 * SPDX-FileCopyrightText: 2026 Tharen Emmanuel Candi <candi.tharen@gmail.com>
 * SPDX-License-Identifier: LGPL-2.1-only
 */

/**
 * @ingroup     drivers_arducam_mini
 * @{
 *
 * @file
 * @brief       ArduCAM Mini 2MP Plus driver: ArduChip (SPI), capture, FIFO read
 *
 * @author      Tharen Emmanuel Candi <candi.tharen@gmail.com>
 *
 * @}
 */

#include <errno.h>            /* ENODEV, EIO, EINVAL, EBUSY, ENXIO */
#include <assert.h>
#include <stdbool.h>
#include "container.h"        /* ARRAY_SIZE */
#include "irq.h"
#include "log.h"
#include "ztimer.h"           /* ztimer_sleep */

#include "arducam_mini.h"
#include "arducam_mini_internal.h"
#include "arducam_mini_params.h"   /* CONFIG_ARDUCAM_MINI_* */
#include "arducam_mini_ov2640_regs.h"

#define ENABLE_DEBUG 0
#include "debug.h"




/**
 * SPI protocol (ArduChip CPLD - the FIFO controller)
 * ==================================================
 * - SPI mode 0, max 8 MHz (we default to 5 MHz)
 * - Every transaction is 2 bytes: [command][data]
 *
 *   Command byte:  bit 7    = R/W flag (1 = write, 0 = read)
 *                  bits 6:0 = register address
 *
 *   Write reg 0x07: send 0x07 | 0x80 = 0x87, then the value
 *   Read  reg 0x07: send 0x07 & 0x7F = 0x07, then a dummy 0x00
 *
 * - SPI is full-duplex: every clock shifts a bit out AND in.
 *   You cannot read without writing, so reads send a dummy 0x00
 *   to generate clocks. Response byte layout:
 *     byte 0 received while sending command  -> garbage
 *     byte 1 received while sending dummy    -> register value
 * - Key registers: 0x00 test, 0x04 FIFO ctrl, 0x07 CPLD reset,
 *   0x41 trigger/status, 0x42-0x44 FIFO length (23-bit),
 *   0x3C burst FIFO read command
 * - Bus etiquette: acquire -> transfer -> release per transaction.
 *   Never hold the bus between public API calls.
 */

/**
 * I2C (SCCB) protocol (OV2640 sensor)
 * ===================================
 * - Standard I2C at 7-bit address 0x30 (SCCB is close enough to
 *   I2C that the regular periph API works)
 * - Register write: one transaction of 2 bytes [reg][val]
 * - Register read: TWO transactions back to back:
 *     1. write [reg]          (tell the sensor which register)
 *     2. read 1 byte          (receive its value)
 * - Register map is banked: writing 0xFF = 0x00 / 0x01 selects
 *   the bank. The chip ID registers (0x0A/0x0B) live in bank 1.
 *   Always re-select the bank before touching banked registers.
 * - The sensor seems to need ~1 ms to digest each transaction
 *   (Bellotto's code sleeps after every transfer) - keep this in
 *   the I2C helpers (for now)
 * - Errors: i2c_write_bytes/i2c_read_byte return non-zero on
 *   NACK -> map to -EIO. A wrong chip ID is -ENODEV.
 */

/* ------------------------------------------------------------------ */
/* private helpers                                                     */
/* ------------------------------------------------------------------ */

//helper functions. 
static int _i2c_write_reg(const arducam_mini_t *dev, uint8_t reg, uint8_t val) {
    
    uint8_t data[2] = { reg, val };
    i2c_acquire(dev->params.i2c);
    int ret = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr,
        data, 2, 0 );
    
    i2c_release(dev->params.i2c);
    if (ret < 0) {
        // raw periph error before we flatten it to -EIO (-ENXIO = address nack)
        DEBUG("[arducam_mini] i2c write reg 0x%02x: raw err %d\n", reg, ret);
    }


    ztimer_sleep(ZTIMER_MSEC, 1); //todo  check if this is needed
    
    return (ret < 0) ? -EIO : 0;
    
}


static int _i2c_read_reg(const arducam_mini_t *dev, uint8_t reg, uint8_t *val) {
    
    i2c_acquire(dev->params.i2c);    
    int ret = i2c_write_byte(dev->params.i2c, dev->params.i2c_addr, reg, 0);
    if (ret == 0) {
        ret = i2c_read_byte(dev->params.i2c, dev->params.i2c_addr, val, 0);
    }
    i2c_release(dev->params.i2c);
    if (ret < 0) {
        DEBUG("[arducam_mini] i2c read reg 0x%02x: raw err %d\n", reg, ret);
    }

    ztimer_sleep(ZTIMER_MSEC, 1); //todo  check if this is needed
    
    return (ret < 0) ? -EIO : 0;
    
}


static uint8_t _spi_read_reg(const arducam_mini_t *dev, uint8_t reg)
{
    assert(dev);
    assert(!(reg & ARDUCAM_MINI_WRITE_BIT));   // pass the plain address, not cmd

    spi_acquire(dev->params.spi, dev->params.spi_cs,
                SPI_MODE_0, dev->params.spi_clk);
    // riot helper for [reg][data] chips: sends reg, then the dummy 0x00,
    // and returns the byte clocked in during the dummy (= register value)
    uint8_t val = spi_transfer_reg(dev->params.spi, dev->params.spi_cs, reg, 0x00);
    spi_release(dev->params.spi);

    return val;
}

// set reg to val.
static void _spi_write_reg(const arducam_mini_t *dev, uint8_t reg, uint8_t val) {
    //0 is for the address, 1 is for the value.
    // and we have to | with write bit, so the on wire address byte is correct. 
    //bit 7   = R/W flag:  1 = write, 0 = read
    //bits 6:0 = register address
    //
    assert(dev);
    assert(!(reg & ARDUCAM_MINI_WRITE_BIT));   // we add the write bit here

    spi_acquire(dev->params.spi, dev->params.spi_cs,
                SPI_MODE_0, dev->params.spi_clk);
    spi_transfer_reg(dev->params.spi, dev->params.spi_cs,
                     reg | ARDUCAM_MINI_WRITE_BIT, val);
    spi_release(dev->params.spi);

}

static int _apply_table(const arducam_mini_t *dev, const struct sensor_reg table[]) {
    
    const struct sensor_reg *entry = table;

    //while not bank switch or the sentinal (0xff, 0xff)
    while (!(entry->reg == 0xFF && entry->val == 0xFF)) {
        
        int ret = _i2c_write_reg(dev, entry->reg, entry->val);
        if (ret < 0)
            return ret;
        entry ++;
    }
    return 0;
}

// largest DSP pre-divider exponent n (in / 2^n, n <= 3) that still leaves
// at least out pixels for the zoom stage, which can only scale down.
// reproduces the arducam uxga tables: 1600 -> 320 gives /4 (0x92),
// 1600 -> 160 or 80 gives /8 (0x9b)
static uint8_t _divider(uint16_t in, uint16_t out) {
    uint8_t n = 0;
    while (n < 3 && (in >> (n + 1)) >= out) {
        n++;
    }
    return n;
}

// sensor readout -> dsp input size (what the readout table sets HSIZE / VSIZE to)
static void _readout_size(arducam_mini_readout_t ro, uint16_t *in_w, uint16_t *in_h) {
    switch (ro) {
        case ARDUCAM_MINI_READOUT_CIF:  *in_w = 400;  *in_h = 296;  break;
        case ARDUCAM_MINI_READOUT_SVGA: *in_w = 800;  *in_h = 600;  break;
        default:                        *in_w = 1600; *in_h = 1200; break;
    }
}

// one supported mode = one row of the conformance bench's size table (set 7
// in tests/drivers/arducam_mini/conformance.c), all passed on the 0x42, see
// issue #2. win 0 = whole readout, otherwise a centred dsp window (crop).
// the divider is _divider() on the window (or the readout), LP_DP on when a
// divider is used, unless ARDUCAM_MINI_SZ_NO_LPDP
typedef struct {
    uint8_t format;         // arducam_mini_format_t
    uint8_t readout;        // arducam_mini_readout_t
    uint8_t fov;            // arducam_mini_fov_t
    uint8_t flags;          // ARDUCAM_MINI_SZ_*
    uint16_t width, height;
    uint16_t win_w, win_h;
} arducam_mini_size_t;

// LP_DP + /8 at zoom 0.8 adds a constant filler line at the end of the frame
// (80x60 svga, 160x120 uxga). without LP_DP the frame is exact, same picture
#define ARDUCAM_MINI_SZ_NO_LPDP     0x01

#define SZ_FULL(f, w, h, ro, fl)    { ARDUCAM_MINI_FMT_##f, ARDUCAM_MINI_READOUT_##ro, \
                                      ARDUCAM_MINI_FOV_FULL, fl, w, h, 0, 0 }
#define SZ_CROP(w, h, ro, ww, wh)   { ARDUCAM_MINI_FMT_RGB565, ARDUCAM_MINI_READOUT_##ro, \
                                      ARDUCAM_MINI_FOV_CROP, 0, w, h, ww, wh }

// why not more (all measured, issue #2):
// - zoom below 0.5 gives an extra filler line and the last pixels of the
//   picture are wrong -> no full-fov 40x32 / 48x36 or 80x60 uxga, the tiny
//   sizes are crops instead (60 % of the width in every readout)
// - the width after the divider must be a multiple of 4 (50 / 98 px broke)
// - cif without a divider (320x240, 352x288) shows line artefacts
// - jpeg at cif shows half the view, stretched and ~13x too dark
static const arducam_mini_size_t _sizes[] = {
    SZ_CROP(40, 32, CIF, 240, 192),                     // /4
    SZ_CROP(40, 32, SVGA, 480, 384),                    // /8
    SZ_CROP(48, 36, CIF, 240, 180),                     // /4
    SZ_CROP(48, 36, SVGA, 480, 360),                    // /8
    SZ_FULL(RGB565, 80, 60, CIF, 0),                    // /4
    SZ_FULL(RGB565, 80, 60, SVGA, ARDUCAM_MINI_SZ_NO_LPDP),    // /8
    SZ_FULL(RGB565, 160, 120, CIF, 0),                  // /2
    SZ_FULL(RGB565, 160, 120, SVGA, 0),                 // /4
    SZ_FULL(RGB565, 160, 120, UXGA, ARDUCAM_MINI_SZ_NO_LPDP),  // /8
    SZ_FULL(RGB565, 176, 144, CIF, 0),                  // /2
    SZ_FULL(RGB565, 176, 144, SVGA, 0),                 // /4
    SZ_FULL(RGB565, 176, 144, UXGA, 0),                 // /8
    SZ_FULL(RGB565, 320, 240, SVGA, 0),                 // /2
    SZ_FULL(RGB565, 320, 240, UXGA, 0),                 // /4
    SZ_FULL(RGB565, 352, 288, SVGA, 0),                 // /2
    SZ_FULL(RGB565, 352, 288, UXGA, 0),                 // /4
    SZ_FULL(JPEG, 160, 120, SVGA, 0),
    SZ_FULL(JPEG, 160, 120, UXGA, 0),
    SZ_FULL(JPEG, 176, 144, SVGA, 0),
    SZ_FULL(JPEG, 176, 144, UXGA, 0),
    SZ_FULL(JPEG, 320, 240, SVGA, 0),
    SZ_FULL(JPEG, 320, 240, UXGA, 0),
    SZ_FULL(JPEG, 352, 288, SVGA, 0),
    SZ_FULL(JPEG, 352, 288, UXGA, 0),
    SZ_FULL(JPEG, 640, 480, SVGA, 0),
    SZ_FULL(JPEG, 640, 480, UXGA, 0),
    SZ_FULL(JPEG, 800, 600, SVGA, 0),                   // 1:1
    SZ_FULL(JPEG, 800, 600, UXGA, 0),
    SZ_FULL(JPEG, 1024, 768, UXGA, 0),                  // 1:1 from here on
    SZ_FULL(JPEG, 1280, 1024, UXGA, 0),
    SZ_FULL(JPEG, 1600, 1200, UXGA, 0),
};

// the table row for a mode, NULL if the combination isnt supported
static const arducam_mini_size_t *_size_lookup(const arducam_mini_mode_t *mode) {
    for (unsigned i = 0; i < ARRAY_SIZE(_sizes); i++) {
        const arducam_mini_size_t *sz = &_sizes[i];
        if (sz->format == mode->format && sz->readout == mode->readout &&
            sz->fov == mode->fov &&
            sz->width == mode->width && sz->height == mode->height) {
            return sz;
        }
    }
    return NULL;
}

// program the dsp for a table row: image window (crop rows only), divider,
// zoom, plus the two dsp extras the arducam jpeg size tables set. holds the
// dvp (output) in reset while changing, then releases it
static int _set_scaler(const arducam_mini_t *dev, const arducam_mini_size_t *sz) {

    uint16_t w = sz->width, h = sz->height;
    uint16_t in_w, in_h;
    _readout_size(sz->readout, &in_w, &in_h);
    // a crop window is the input of the divider and zoom
    uint16_t win_w = sz->win_w ? sz->win_w : in_w;
    uint16_t win_h = sz->win_h ? sz->win_h : in_h;
    uint8_t div_h = _divider(win_w, w), div_v = _divider(win_h, h);
    // LP_DP only when a divider is used: the 1:1 jpeg tables (1024x768 and up)
    // have CTRLI = 0x00, every divided one has bit 7 set
    bool lp_dp = (div_h || div_v) && !(sz->flags & ARDUCAM_MINI_SZ_NO_LPDP);
    uint8_t ctrli = (lp_dp ? OV2640_CTRLI_LP_DP : 0) | (div_v << 3) | div_h;
    uint16_t outw = w / 4, outh = h / 4;   /* zoom registers count in 4s */

    if (_i2c_write_reg(dev, OV2640_REG_BANK_SEL, OV2640_BANK_DSP) ||
        // not every path gets here with the dvp already in reset (raw uxga
        // has no readout table), so hold it ourselves
        _i2c_write_reg(dev, OV2640_REG_RESET, OV2640_RESET_DVP)) {
        return -EIO;
    }

    // crop: centred window inside the readout. sizes real / 4, offsets in
    // pixels, the high bits in VHYX / TEST (same layout as openmv / linux)
    if (sz->win_w) {
        uint16_t xo = (in_w - win_w) / 2, yo = (in_h - win_h) / 2;
        if (_i2c_write_reg(dev, OV2640_REG_HSIZE, (win_w >> 2) & 0xFF) ||
            _i2c_write_reg(dev, OV2640_REG_VSIZE, (win_h >> 2) & 0xFF) ||
            _i2c_write_reg(dev, OV2640_REG_XOFFL, xo & 0xFF) ||
            _i2c_write_reg(dev, OV2640_REG_YOFFL, yo & 0xFF) ||
            _i2c_write_reg(dev, OV2640_REG_VHYX, ((win_h >> 10) & 1) << 7 | ((yo >> 8) & 7) << 4 |
                                                 ((win_w >> 10) & 1) << 3 | ((xo >> 8) & 7)) ||
            _i2c_write_reg(dev, OV2640_REG_TEST, ((win_w >> 11) & 1) << 7)) {
            return -EIO;
        }
    }

    if (_i2c_write_reg(dev, OV2640_REG_CTRLI, ctrli) ||
        _i2c_write_reg(dev, OV2640_REG_ZMOW, outw & 0xFF) ||
        _i2c_write_reg(dev, OV2640_REG_ZMOH, outh & 0xFF) ||
        // arducam's 1280x1024 jpeg table leaves OUTH[8] out here (-> 1280x960)
        _i2c_write_reg(dev, OV2640_REG_ZMHH, ((outh >> 8) & 0x01) << 2 | ((outw >> 8) & 0x03))) {
        return -EIO;
    }

    // jpeg dsp extras, same values as the arducam jpeg size tables.
    // raw keeps what OV2640_RGB565_BASE set (CTRL2 0x3d, R_DVP_SP 0x04).
    // (arducam's 800x600 table has CTRL2 0x35, this gives 0x3d - check on board)
    if (sz->format == ARDUCAM_MINI_FMT_JPEG) {
        uint8_t ctrl2 = (sz->readout == ARDUCAM_MINI_READOUT_UXGA) ? 0x3d : 0x35;
        if (_i2c_write_reg(dev, OV2640_REG_CTRL2, ctrl2)) {
            return -EIO;
        }
        // svga keeps R_DVP_SP 0x04 from JPEG_INIT (its 0x7f entries are
        // commented out, same as in arducam's tables), uxga sizes set it
        if (sz->readout == ARDUCAM_MINI_READOUT_UXGA &&
            _i2c_write_reg(dev, OV2640_REG_R_DVP_SP, (w <= 640) ? 0x04 : 0x02)) {
            return -EIO;
        }
    }

    if (_i2c_write_reg(dev, OV2640_REG_RESET, 0x00)) {
        return -EIO;
    }
    return 0;
}

// one REG04 value for both formats from params hflip / vflip, written after
// all tables (the base tables leave rgb565 at 0xa8 but jpeg at 0x08, so jpeg
// came out mirrored). bench sets 8 + 10 on the 0x42: bit 7 decides the
// horizontal direction (set = the right way round), a vertical flip needs
// bits 6 + 4 together (bit 6 alone swaps the colours, openmv sets both too),
// bits 5 and 3 make no difference. normal 0xa8, hflip 0x28, vflip 0xf8,
// both 0x78
static int _set_orientation(const arducam_mini_t *dev) {

    uint8_t reg04 = OV2640_REG04_BASE;
    if (!dev->params.hflip) {
        reg04 |= OV2640_REG04_HMIRROR;
    }
    if (dev->params.vflip) {
        reg04 |= OV2640_REG04_VFLIP;
    }
    if (_i2c_write_reg(dev, OV2640_REG_BANK_SEL, OV2640_BANK_SENSOR) ||
        _i2c_write_reg(dev, OV2640_REG_REG04, reg04)) {
        return -EIO;
    }
    return 0;
}

// write the sensor registers for a mode and its table row: soft reset ->
// format base table -> readout table if needed -> dsp scaler -> orientation.
// always starts from the reset defaults, so switching modes at runtime works
static int _configure(arducam_mini_t *dev, const arducam_mini_mode_t *mode,
                      const arducam_mini_size_t *sz) {

    const struct sensor_reg *readout_table = NULL;
    switch (sz->readout) {
        case ARDUCAM_MINI_READOUT_SVGA: readout_table = OV2640_READOUT_SVGA; break;
        case ARDUCAM_MINI_READOUT_CIF:  readout_table = OV2640_READOUT_CIF;  break;
        default:                        readout_table = OV2640_READOUT_UXGA; break;
    }

    // soft reset first, so every mode starts from the power-up defaults.
    // the rgb565 base table doesnt write the sensor window (0x03, 0x17-0x1a,
    // 0x32), so after a jpeg / svga / cif mode, raw uxga kept the old window
    // and the sensor sent no frames at all (0x42, 18 oct). errors only logged,
    // like in _probe_sensor: some sensors dont ack the reset write
    if (_i2c_write_reg(dev, OV2640_REG_BANK_SEL, OV2640_BANK_SENSOR) ||
        _i2c_write_reg(dev, OV2640_REG_COM7, OV2640_COM7_RESET)) {
        DEBUG("[arducam_mini] soft reset write failed (ignored)\n");
    }
    ztimer_sleep(ZTIMER_MSEC, ARDUCAM_MINI_RESET_DELAY_MS);

    if (mode->format == ARDUCAM_MINI_FMT_JPEG) {
        //jpeg init
        if (_apply_table(dev, OV2640_JPEG_INIT) < 0) { DEBUG("[arducam_mini] jpeg_init failed\n"); return -EIO; }
        // then set colour space (yuv422)
        if (_apply_table(dev, OV2640_YUV422) < 0) { DEBUG("[arducam_mini] yuv422 failed\n"); return -EIO; }
        // switch to jpeg output (also sets bank 1 + COM10 defaults)
        if (_apply_table(dev, OV2640_JPEG) < 0) { DEBUG("[arducam_mini] jpeg failed\n"); return -EIO; }
        // JPEG_INIT leaves an odd dsp input size, so jpeg always needs a readout table
    }
    else {
        // rgb565 base already runs uxga, only svga / cif need a switch
        if (_apply_table(dev, OV2640_RGB565_BASE) < 0) { DEBUG("[arducam_mini] rgb565 base failed\n"); return -EIO; }
        if (sz->readout == ARDUCAM_MINI_READOUT_UXGA) {
            readout_table = NULL;
        }
    }
    DEBUG("[arducam_mini] base tables ok\n");

    if (readout_table && _apply_table(dev, readout_table) < 0) {
        DEBUG("[arducam_mini] readout table failed\n");
        return -EIO;
    }
    if (_set_scaler(dev, sz) < 0) {
        DEBUG("[arducam_mini] scaler failed\n");
        return -EIO;
    }
    if (_set_orientation(dev) < 0) {
        DEBUG("[arducam_mini] orientation failed\n");
        return -EIO;
    }

    // datasheet: register settings take up to 300 ms to settle, and
    // aec/awb need a few frames after a timing change
    ztimer_sleep(ZTIMER_MSEC, ARDUCAM_MINI_SETTLE_DELAY_MS);

    // clear the fifo
    _spi_write_reg(dev, ARDUCAM_MINI_REG_FIFO, ARDUCAM_MINI_FIFO_CLEAR_MASK);

    dev->mode = *mode;
    DEBUG("[arducam_mini] mode %ux%u fmt %d readout %d fov %d ok\n",
          mode->width, mode->height, (int)mode->format, (int)mode->readout,
          (int)mode->fov);
    return 0;
}

// soft reset the sensor and check its chip id
static int _probe_sensor(const arducam_mini_t *dev) {

    uint8_t vid, pid;

    // soft reset. errors here are only logged, not returned: some sensors
    // dont ack the reset write (they reset before the ack). the id read below
    // is the real test of whether the sensor is there
    if (_i2c_write_reg(dev, OV2640_REG_BANK_SEL, OV2640_BANK_SENSOR)) {
        DEBUG("[arducam_mini] probe: bank select before reset failed (ignored)\n");
    }
    if (_i2c_write_reg(dev, OV2640_REG_COM7, OV2640_COM7_RESET)) {
        DEBUG("[arducam_mini] probe: reset write failed (ignored)\n");
    }
    ztimer_sleep(ZTIMER_MSEC, ARDUCAM_MINI_RESET_DELAY_MS);

    // each step on its own so we can see where it fails
    /* reset may have changed the bank */
    if (_i2c_write_reg(dev, OV2640_REG_BANK_SEL, OV2640_BANK_SENSOR)) {
        DEBUG("[arducam_mini] probe: bank select failed\n");
        return -EIO;
    }
    if (_i2c_read_reg(dev, OV2640_CHIPID_HIGH, &vid)) {
        DEBUG("[arducam_mini] probe: vid read failed\n");
        return -EIO;
    }
    if (_i2c_read_reg(dev, OV2640_CHIPID_LOW, &pid)) {
        DEBUG("[arducam_mini] probe: pid read failed\n");
        return -EIO;
    }
    DEBUG("[arducam_mini] vid=0x%02x pid=0x%02x\n", vid, pid);

    if (vid != OV2640_EXPECTED_VID ||
        (pid != OV2640_EXPECTED_PID_1 && pid != OV2640_EXPECTED_PID_2)) {
        return -ENODEV;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* public API (documented in arducam_mini.h)                           */
/* ------------------------------------------------------------------ */

int arducam_mini_init(arducam_mini_t *dev, const arducam_mini_params_t *params) {

    // assert dev paramas not null and set dev.params = params
    assert(!irq_is_in());
    assert (dev != NULL);
    assert (params != NULL);
    dev->params = *params;
    dev->mode = params->mode;
    dev->state = ARDUCAM_MINI_IDLE;
    DEBUG("[arducam_mini] init start\n");

    // unsupported combinations are caught before touching the hardware
    const arducam_mini_size_t *sz = _size_lookup(&params->mode);
    if (sz == NULL) {
        return -EINVAL;
    }

    // 1. chip select. riot already ran i2c_init / spi_init for every bus at
    // boot (periph_common/init.c), so drivers only set up their own cs pin
    if (spi_init_cs(params->spi, params->spi_cs) != SPI_OK) {
        LOG_ERROR("[arducam_mini] %s\n", "cannot init spi chip select");
        return -ENXIO;
    }
    DEBUG("[arducam_mini] cs ok\n");

    // cs high resets the arduchip's spi logic. spi_init_cs leaves cs inactive
    // (high), so wait ~10 ms and do one throwaway read: if a warm reset left the
    // arduchip mid-transfer, this brings it back in step before we trust a read
    ztimer_sleep(ZTIMER_MSEC, ARDUCAM_MINI_CS_SETTLE_MS);
    (void)_spi_read_reg(dev, ARDUCAM_MINI_REG_TEST1);

    // 2. arduchip reset and sanity check, a few times (bounded)
    // resetting involves setting bit 7 of 0x07
    //cpld is the Complex Programmable Logic Device (FIFO controller)
    // it buffers frames, bridges protocol (spi to sensor paral bus)
    // and it does flow control. 
    //sanity check: write TEST_1, read it back. print the read back every try
    uint8_t test_1_val = 0x55;
    uint8_t readback = 0;
    unsigned spi_tries = 0;
    while (spi_tries < ARDUCAM_MINI_SPI_TEST_RETRIES) {
        spi_tries++;
        _spi_write_reg(dev, ARDUCAM_MINI_REG_CPLD_RESET, ARDUCAM_MINI_CPLD_RESET_MASK);
        ztimer_sleep(ZTIMER_MSEC, ARDUCAM_MINI_RESET_DELAY_MS);
        _spi_write_reg(dev, ARDUCAM_MINI_REG_CPLD_RESET, 0x00); //release cpld from reset
        ztimer_sleep(ZTIMER_MSEC, ARDUCAM_MINI_RESET_DELAY_MS);

        _spi_write_reg(dev, ARDUCAM_MINI_REG_TEST1, test_1_val);
        readback = _spi_read_reg(dev, ARDUCAM_MINI_REG_TEST1);
        DEBUG("[arducam_mini] spi test try %u: wrote 0x%02x read 0x%02x\n",
              spi_tries, test_1_val, readback);
        if (readback == test_1_val) {
            break;
        }
    }
    if (readback != test_1_val) {
        LOG_ERROR("[arducam_mini] spi test failed after %u tries, last read 0x%02x\n",
                  spi_tries, readback);
        return -ENODEV;
    }
    DEBUG("[arducam_mini] spi test ok\n");

    // diagnostics every boot: arduchip version, gpio write (sensor reset /
    // standby / power pins as last written) and gpio read (pin levels).
    // if 0x45 shows the sensor held in reset or powered off after a bad warm
    // reset, a garbage write to 0x06 did it (mechanism A)
    DEBUG("[arducam_mini] regs: ver(0x40)=0x%02x gpio_w(0x06)=0x%02x gpio_r(0x45)=0x%02x\n",
          _spi_read_reg(dev, ARDUCAM_MINI_REG_VERSION),
          _spi_read_reg(dev, ARDUCAM_MINI_REG_GPIO_WRITE),
          _spi_read_reg(dev, ARDUCAM_MINI_REG_GPIO_READ));

    // 3. probe/search for  the ov2640 sensor 
    // change from reference code - we do not want to hang forever waiting, 
    // application should decide if it cant find here. 
    /* 3. probe the OV2640 sensor (M2) */
    // like si70xx / amg88xx: a few tries, and only sleep after a failed one
    // (the soft reset in _probe_sensor already waits on the success path)
    int ret = -ENODEV;
    unsigned tries = 0;
    while (tries < CONFIG_ARDUCAM_MINI_PROBE_RETRIES) {
        tries++;
        ret = _probe_sensor(dev);
        if (ret == 0) {
            break;
        }
        ztimer_sleep(ZTIMER_MSEC, ARDUCAM_MINI_RESET_DELAY_MS);
    }
    if (ret) {
        LOG_ERROR("[arducam_mini] sensor not found after %u tries: %d\n", tries, ret);
        return ret;
    }
    // if this is regularly > 1 the power/wiring issue is still there
    DEBUG("[arducam_mini] sensor found (try %u)\n", tries);
    
    // 4. configure the sensor for params->mode
    // base table for the format, readout table, then the dsp scaler for the
    // size table row (no per-size register tables any more, see _configure)
    ret = _configure(dev, &params->mode, sz);
    if (ret < 0) {
        return ret;
    }

    // fin 
    DEBUG("[arducam_mini] init done\n");
    return 0;

}

int arducam_mini_set_mode(arducam_mini_t *dev, const arducam_mini_mode_t *mode) {
    // same register writes as init, just without the bus / cpld / probe part
    assert(!irq_is_in());
    assert(dev != NULL);
    assert(mode != NULL);

    if (dev->state != ARDUCAM_MINI_IDLE) {
        return -EBUSY;
    }
    const arducam_mini_size_t *sz = _size_lookup(mode);
    if (sz == NULL) {
        return -EINVAL;
    }
    return _configure(dev, mode, sz);
}


/* ------------------------------------------------------------------ */
/* private helpers: capture + fifo read                                */
/* ------------------------------------------------------------------ */

static uint32_t _read_fifo_length(arducam_mini_t *dev) {
    //  reads regs 0x42, 0x43, 0x44
    // (23-bit value, mask top byte with 0x7F)
    uint8_t len1 = _spi_read_reg(dev, ARDUCAM_MINI_REG_FIFO_SIZE1);  /* low byte */
    uint8_t len2 = _spi_read_reg(dev, ARDUCAM_MINI_REG_FIFO_SIZE2);  /* middle */
    uint8_t len3 = _spi_read_reg(dev, ARDUCAM_MINI_REG_FIFO_SIZE3);  /* high - only 7 bits valid */
    uint32_t len= ((uint32_t)(len3 & 0x7F) << 16) |   /* high 7 bits */
                  ((uint32_t)len2 << 8) |              /* middle 8 bits */
                  ((uint32_t)len1);                    /* low 8 bits */
    return len;
}


//read image from fifo
/// a burst read is a continuous SPI transaction where CS stays low (this makes it blocking.. nothing else can use the SPI) while
// clocking out multiple bytes. The CPLD auto-increments its FIFO read
// pointer internally, so you just keep reading until you have the frame.
// Scan for JPEG markers (FF D8 start, FF D9 end) to extract the image
// from any padding bytes in the FIFO
// fifo_len comes from _wait_done now (it reads the length regs), not from here
static int32_t _read_fifo_burst(arducam_mini_t *dev, uint8_t *buf, size_t max_len, uint32_t fifo_len) {

    // 1. start burst read, first acquire, then start the cont transfer
    spi_acquire(dev->params.spi, dev->params.spi_cs, SPI_MODE_0, dev->params.spi_clk);
    spi_transfer_byte(dev->params.spi, dev->params.spi_cs, true, ARDUCAM_MINI_CMD_BURST_READ);
   
    // first byte isnt actually garbage on the mini plus, it can be the ff of ff d8
    // so keep it as prev_byte like bellotto does (temp -> temp_last), keep CS low
    uint8_t prev_byte = spi_transfer_byte(dev->params.spi, dev->params.spi_cs, true, 0x00);
    fifo_len --;
    // 2. read bytes and find jpeg header and end-marker ( ff d8)
    uint8_t is_header = false;
    bool is_end = false;

    size_t buf_idx = 0;
    for (uint32_t i = 0; i < fifo_len; i++) {

        uint8_t cur_byte = spi_transfer_byte(dev->params.spi, dev->params.spi_cs, true, 0x00);
        
        /* start of JPEG: FF D8 */
        // only if we havent started yet. ff d8 cant legally show up inside a jpeg
        // but corrupted data can have one, and then we'd write an extra ff into buf
        if (!is_header && cur_byte == ARDUCAM_MINI_JPEG_SOI && prev_byte == ARDUCAM_MINI_JPEG_MARKER_PREFIX) {
            is_header = true;
            buf[buf_idx++] = prev_byte;

        }

        // get rid of nesting i think it works here?
        if (buf_idx >= max_len) {
            // same cs thing as below - raise it before we bail
            spi_transfer_byte(dev->params.spi, dev->params.spi_cs, false, 0x00);
            spi_release(dev->params.spi);
            return -ENOBUFS;   /* caller's buffer too small */
        }
    
        if (is_header) 
            buf[buf_idx++] = cur_byte;

        /* end of JPEG: FF D9 */
        // only count it once we've actually seen the start, otherwise
        // a stray ff d9 in the padding ends the read early and we return 0
        if (is_header && cur_byte == ARDUCAM_MINI_JPEG_EOI && prev_byte == ARDUCAM_MINI_JPEG_MARKER_PREFIX) {
            is_end = true;
            break;
        }

        
        prev_byte = cur_byte;

    }

    // every byte above used cont = true, which keeps CS low until a transfer with
    // cont = false. spi_release doesnt promise to raise it, so clock one throwaway
    // byte with false. losing a fifo byte is fine, we clear the fifo after anyway.
    // (raw path doesnt need this, its last transfer already uses false)
    spi_transfer_byte(dev->params.spi, dev->params.spi_cs, false, 0x00);
    spi_release(dev->params.spi);

    /* no FF D8 ... FF D9 in the whole FIFO read: the frame is garbage
       (desync, wrong format, or a corrupt capture), not a valid jpeg.
       without the end marker we'd hand back the whole fifo as a "jpeg".
       keep the api contract strict: positive length or negative errno */
    if (!is_end) {
        return -EIO;
    }

    return buf_idx;
}


//raw frames are fixed size per resolution 
//153,600 / 38,400 / 9,600 bytes)

static int32_t _read_fifo_raw(arducam_mini_t *dev, uint8_t *buf, uint32_t fifo_len) {

    uint32_t frame_size = arducam_mini_frame_size(&dev->mode);
    
    // desync check: raw has no markers, so the length is our only corruption
    // detector. the fifo holds the frame plus a few pad bytes (uc-474 +8,
    // uc-c78 +16). an extra or missing line is at least 80 bytes (40x32 is
    // the narrowest supported size), so it cant hide in the pad
    if (fifo_len < frame_size || fifo_len > frame_size + ARDUCAM_MINI_FIFO_PAD_MAX) {
        DEBUG("[arducam_mini] raw fifo %lu bytes, frame %lu\n",
              (unsigned long)fifo_len, (unsigned long)frame_size);
        return -EIO;
    }
    
    // spi_transfer_regs = burst command with cs held low, then frame_size
    // bytes with cont = false, so cs goes high at the end. same as before,
    // just the riot helper (periph_common/spi.c)
    spi_acquire(dev->params.spi, dev->params.spi_cs, SPI_MODE_0, dev->params.spi_clk);
    spi_transfer_regs(dev->params.spi, dev->params.spi_cs,
                      ARDUCAM_MINI_CMD_BURST_READ, NULL, buf, frame_size);
    spi_release(dev->params.spi);
                                

    return frame_size;

}




// two helpers: 
//poll done bit, read + check length, return length

//- _wait_done (:413): it clears the FIFO itself on timeout, but not on the -EIO path. 
//Take the clear out completely, so helpers have no side effects and wait does all the cleanup.
static int32_t _wait_done(arducam_mini_t *dev, uint32_t timeout_ms)  {

    // poll cap done mask with timeout , for read
    //  we need to read the status register which is the trig
    // it tells u if done, full, empty,     
    uint32_t start = ztimer_now(ZTIMER_MSEC);
    while (!(_spi_read_reg(dev, ARDUCAM_MINI_REG_TRIG) & ARDUCAM_MINI_CAP_DONE_MASK) ) {
        if ((ztimer_now(ZTIMER_MSEC) - start) > timeout_ms) {
            
            return -ETIMEDOUT;
        }
            
        ztimer_sleep(ZTIMER_MSEC, CONFIG_ARDUCAM_MINI_POLL_MS); //wait between each poll
    }


    uint32_t fifo_len = _read_fifo_length(dev);

    if (fifo_len == 0 || fifo_len >= ARDUCAM_MINI_FIFO_MAX_SIZE) 
        return -EIO;
    return fifo_len;

}
static int32_t _read_fifo(arducam_mini_t *dev, uint8_t *buf, size_t max_len, uint32_t fifo_len ) {
    int32_t len;
    switch (dev->mode.format) {

        //jpeg specific : read the fifo burst now that we know its ready.
        //because it has to do stuff like finding the header n stuff
        //probs will rename it 
        case ARDUCAM_MINI_FMT_JPEG:
            len = _read_fifo_burst(dev, buf, max_len, fifo_len);
            break;

        case ARDUCAM_MINI_FMT_RGB565:
            // simpler case - always same size.
            len = _read_fifo_raw(dev, buf, fifo_len);
            break;

        default:
            len = -EINVAL;
            break;
    }

    return len;
} 



/* ------------------------------------------------------------------ */
/* public API: capture                                                 */
/* ------------------------------------------------------------------ */

int arducam_mini_capture_start(arducam_mini_t *dev) {
    // kick off a capture and return straight away, never blocks
    assert(!irq_is_in());
    assert (dev != NULL);
    if (dev->state != ARDUCAM_MINI_IDLE)
        return -EBUSY;
    
    //start capture by writing to fifo start mask
    _spi_write_reg(dev, ARDUCAM_MINI_REG_FIFO, ARDUCAM_MINI_FIFO_START_MASK);

    dev->state = ARDUCAM_MINI_CAPTURING;
    return 0;

}
int32_t arducam_mini_capture_wait(arducam_mini_t *dev, uint8_t *buf, 
                                size_t max_len, uint32_t timeout_ms) {

    // wait for the capture start kicked off, read it out, then always clean up (fifo clear + idle)

    assert(!irq_is_in());
    assert (dev != NULL);
    if (dev->state != ARDUCAM_MINI_CAPTURING)
        return -EINVAL;
    
    int32_t ret;
    // if buffer isnt big enough, abort capture (clean up and back to idle)
    size_t frame_size = arducam_mini_frame_size(&dev->mode);
    if (frame_size != 0 && max_len < frame_size) {
        ret =  -ENOBUFS;   /* raw: known up front, nothing clocked out */
        // the capture start kicked off is still being written into the fifo.
        // let it finish before the clear in out, otherwise the cpld can finish
        // writing after we cleared and the next capture reads a stale/half frame.
        // result doesnt matter, if it times out we still clear and go idle
        (void)_wait_done(dev, timeout_ms);
        goto out;

    }

    int32_t fifo_len = _wait_done(dev, timeout_ms);
    if (fifo_len < 0) {
        ret = fifo_len;
        goto out;
    }
        
    
    int32_t len = _read_fifo(dev, buf, max_len, fifo_len);
    ret = len;

    out:
    // cleanup, every path after the state check ends here.
    // clear the fifo even if the read failed (e.g -ETIMEDOUT / -EIO), otherwise the next
    // capture starts with the old frame still sitting in there
    _spi_write_reg(dev, ARDUCAM_MINI_REG_FIFO, ARDUCAM_MINI_FIFO_CLEAR_MASK);
    dev->state=ARDUCAM_MINI_IDLE;
    return ret;


 }



int32_t arducam_mini_capture(arducam_mini_t *dev , uint8_t *buf, 
                                size_t max_len, uint32_t timeout_ms) {
     
    int32_t ret;
    ret = arducam_mini_capture_start(dev);
    if (ret < 0)
        return ret;
    
    return arducam_mini_capture_wait(dev, buf, max_len, timeout_ms);

}
