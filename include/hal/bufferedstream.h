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

#pragma once 

#include <cstdint>
#include <type_traits>

#include "container/fifo.h"

// todo: Mode: BitMode oder ByteMode

struct ByteMode {
};
struct BitMode {
};

template<typename Device, uint16_t Size, typename LT, typename Mode = ByteMode>
class BufferedStream {
public:
    typedef Device device_type;
    typedef LT line_terminator_type;

    typedef typename std::conditional<Size <= 255, uint8_t, uint16_t>::type size_type;
    static constexpr const size_type  size = Size;
    
    template<uint16_t Baudrate = 0>
    static void init() {
        Device::template init<Baudrate>();
    }

    static bool put(std::byte v) {
        return fifo.push_back(v);
    }
    static void periodic() {
        if (auto v = fifo.pop_front()) {
            Device::put(*v);
        }
    }
    static constexpr auto rateProcess = periodic;
private:
    inline static std::FiFo<std::byte, Size> fifo;
};