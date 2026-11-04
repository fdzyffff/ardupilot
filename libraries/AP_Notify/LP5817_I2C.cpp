/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* LED driver for LP5817 (3-channel I2C RGB LED driver, 7-bit addr 0x2D) */

#include "LP5817_I2C.h"

#if AP_NOTIFY_LP5817_I2C_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define LP5817_I2C_LED_BRIGHT  255    // full brightness
#define LP5817_I2C_LED_MEDIUM  170    // medium brightness
#define LP5817_I2C_LED_DIM     85     // dim
#define LP5817_I2C_LED_OFF     0      // off

#define LP5817_I2C_ADDR        0x2D   // 7-bit I2C address

// Register map (TI LP5817 datasheet, ZHCSXY5A table 7-6)
enum class Register {
    CHIP_EN        = 0x00, // bit0: chip enable
    DEV_CONFIG0    = 0x01, // bit0: MAX_CURRENT (0=25.5mA, 1=51mA)
    DEV_CONFIG1    = 0x02, // bit2/1/0: OUT2/1/0 enable
    DEV_CONFIG2    = 0x03, // fade
    DEV_CONFIG3    = 0x04, // exponential dimming
    SHUTDOWN_CMD   = 0x0D, // write 0x33 to enter shutdown
    RESET_CMD      = 0x0E, // write 0xCC to reset all registers
    UPDATE_CMD     = 0x0F, // write 0x55 to make device config take effect
    FLAG_CLR       = 0x13, // bit1 TSD_CLR, bit0 POR_CLR (W1C)
    OUT0_DC        = 0x14, // OUT0 (RED) dot current
    OUT1_DC        = 0x15, // OUT1 (GREEN) dot current
    OUT2_DC        = 0x16, // OUT2 (BLUE) dot current
    OUT0_PWM       = 0x18, // OUT0 (RED) manual PWM
    OUT1_PWM       = 0x19, // OUT1 (GREEN) manual PWM
    OUT2_PWM       = 0x1A, // OUT2 (BLUE) manual PWM
    FLAG           = 0x40, // bit1 TSD, bit0 POR (read only)
};

#define LP5817_I2C_CHIP_ENABLE     0x01
#define LP5817_I2C_ALL_OUT_ENABLE  0x07
#define LP5817_I2C_UPDATE_MAGIC    0x55
#define LP5817_I2C_DC_FULL         0xFF  // 100% dot current
#define LP5817_I2C_FLAG_CLR_ALL    0x03  // clear POR and TSD flags

LP5817_I2C::LP5817_I2C(uint8_t bus, uint8_t max_current)
    : RGBLed(LP5817_I2C_LED_OFF, LP5817_I2C_LED_BRIGHT, LP5817_I2C_LED_MEDIUM, LP5817_I2C_LED_DIM)
    , _bus(bus)
    , _max_current(max_current)
    , _need_update(false)
    , _init_done(false)
    , _init_failed(false)
    , _cfg_attempts(0)
{
}

bool LP5817_I2C::init(void)
{
    // Never touch the I2C bus from the main thread during early boot: just
    // obtain the device handle and defer all register access to the I2C
    // periodic callback thread (_timer), which is serialised and
    // timeout-guarded by the I2C manager.
    _dev = hal.i2c_mgr->get_device(_bus, LP5817_I2C_ADDR);

    if (!_dev) {
        // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LP5817: no device handle on bus %u", _bus);
        return false;
    }

    // configure/update at 50Hz inside the I2C callback thread
    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&LP5817_I2C::_timer, void));

    return true;
}

// configure the device registers. Runs in the I2C periodic callback thread.
// The first CHIP_EN write doubles as the presence probe. The sequence follows
// the datasheet programming procedure (8.2.3.2): enable -> max current -> dot
// current -> output enable -> UPDATE_CMD; PWM values are written afterwards by
// _timer as the colour state machine drives them.
bool LP5817_I2C::_configure_device(void)
{
    // enable the device; a successful write confirms a device answers on _bus
    if (!_dev->write_register(uint8_t(Register::CHIP_EN), LP5817_I2C_CHIP_ENABLE)) {
        return false;
    }

    // set maximum current: 0 = 25.5mA, 1 = 51mA
    const uint8_t mc = (_max_current != 0) ? 0x01 : 0x00;
    if (!_dev->write_register(uint8_t(Register::DEV_CONFIG0), mc)) {
        return false;
    }

    // set all three channels to full dot current; brightness is fully handled by PWM
    if (!_dev->write_register(uint8_t(Register::OUT0_DC), LP5817_I2C_DC_FULL) ||
        !_dev->write_register(uint8_t(Register::OUT1_DC), LP5817_I2C_DC_FULL) ||
        !_dev->write_register(uint8_t(Register::OUT2_DC), LP5817_I2C_DC_FULL)) {
        return false;
    }

    // enable all three outputs
    if (!_dev->write_register(uint8_t(Register::DEV_CONFIG1), LP5817_I2C_ALL_OUT_ENABLE)) {
        return false;
    }

    // make the configuration take effect
    if (!_dev->write_register(uint8_t(Register::UPDATE_CMD), LP5817_I2C_UPDATE_MAGIC)) {
        return false;
    }

    // clear POR and TSD flags (write-1-to-clear)
    _dev->write_register(uint8_t(Register::FLAG_CLR), LP5817_I2C_FLAG_CLR_ALL);

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool LP5817_I2C::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb.r = red;
    rgb.g = green;
    rgb.b = blue;
    _need_update = true;
    return true;
}

void LP5817_I2C::_timer(void)
{
    if (!_init_done) {
        if (_init_failed) {
            // device absent (or could not be configured): stay quiet
            return;
        }
        // First tick in the I2C callback thread: configure the device here so
        // boot never blocks on synchronous I2C in AP_Notify::add_backends().
        // Retry briefly to tolerate the LED powering up after the flight
        // controller.
        if (_configure_device()) {
            _init_done = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LP5817: configured OK (bus %u)", _bus);
        } else {
            _cfg_attempts++;
            if (_cfg_attempts >= 20) {   // ~0.4s of 20ms ticks
                _init_failed = true;
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LP5817: no device on bus %u", _bus);
            }
            return;
        }
    }

    if (!_need_update) {
        return;
    }
    _need_update = false;

    write_pwm();
}

// write the three PWM duty registers (OUT0=R, OUT1=G, OUT2=B)
bool LP5817_I2C::write_pwm(void)
{
    return _dev->write_register(uint8_t(Register::OUT0_PWM), rgb.r) &&
           _dev->write_register(uint8_t(Register::OUT1_PWM), rgb.g) &&
           _dev->write_register(uint8_t(Register::OUT2_PWM), rgb.b);
}

#endif  // AP_NOTIFY_LP5817_I2C_ENABLED
