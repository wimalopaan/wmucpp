/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mcu/avr8.h"
#include "mcu/ports.h"

using PortB = AVR::Port<AVR::ATMega328P::PortRegister, AVR::B>;
using Status = AVR::ATMega328P::Status;

class SC {
    inline static constexpr auto status = AVR::getBaseAddr<Status>;
public:
    SC() : v(status()->value.value()) {
        cli();
    }
    ~SC() {
        if (std::toBool(DefaultMcuType::Status::Bits::globalIntEnable & v)) {
            sei();
        }
    }
private:
    DefaultMcuType::Status::Bits v;
};


int main() {
    {
        SC sc;
        while(true) {}
    }
    
}
