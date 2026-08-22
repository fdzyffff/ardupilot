/*
   LP5817 I2C driver

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

   Datasheet: https://www.ti.com/lit/ds/symlink/lp5817.pdf

 */
#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_LP5817_I2C_ENABLED

#include <AP_HAL/I2CDevice.h>
#include "RGBLed.h"

class LP5817_I2C : public RGBLed
{
public:
    LP5817_I2C(uint8_t bus, uint8_t max_current);
    ~LP5817_I2C() { delete _dev; }

    bool init(void) override;
protected:
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) override;

private:
    AP_HAL::I2CDevice *_dev;
    uint8_t _bus;
    uint8_t _max_current;   // 0 = 25.5mA, 1 = 51mA

    void _timer(void);
    bool write_pwm(const uint8_t rgb[3]);
    bool _need_update;
    uint8_t rgb[3];
};

#endif  // AP_NOTIFY_LP5817_I2C_ENABLED
