/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#pragma once

#include <stdint.h>

namespace AVR {

template<uint8_t N, typename MCU = DefaultMcuType>
class Adc final {
    static_assert(N < MCU::Adc::count, "wrong adc number"); 
    static constexpr auto base = getBaseAddr<typename MCU::Adc, N>();

public:
    typedef MCU mcu_type;
    static constexpr const uint8_t number = N;
    Adc() = delete;

    static void init() {
        
    }
    
    static void channel(uint8_t ch) {
        assert(ch < MCU::Adc::template Parameter<0>::numberOfChannels);
    }
    
private:
};

}
