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
#include <std/utility>
#include <std/chrono>

namespace AVR {
    namespace Series0 {
        using namespace std::literals::chrono;

        struct WatchDog final {
            enum class CtrlA1_t: uint8_t {
                off = 0x00,
                c8  = 0x01,
                c16 = 0x02,
                c32 = 0x03,
                c64 = 0x04,
                c128 = 0x05,
                c256 = 0x06,
                c512 = 0x07,
                c1k = 0x08,
                c2k = 0x09,
                c4k = 0x0a,
                c8k = 0x0b,
            };
            enum class CtrlA2_t: uint8_t {
                off = 0x00,
                c8  = 0x01,
                c16 = 0x02,
                c32 = 0x03,
                c64 = 0x04,
                c128 = 0x05,
                c256 = 0x06,
                c512 = 0x07,
                c1k = 0x08,
                c2k = 0x09,
                c4k = 0x0a,
                c8k = 0x0b,
            };
            
            struct per_t {
                CtrlA2_t bits;
                std::chrono::milliseconds timeout;                
            };
            
            
            static inline constexpr std::array<per_t, 11> periodValues {
                per_t{CtrlA2_t::c8, 8_ms},
                per_t{CtrlA2_t::c16, 16_ms},
                per_t{CtrlA2_t::c32, 32_ms},
                per_t{CtrlA2_t::c64, 64_ms},
                per_t{CtrlA2_t::c128, 128_ms},
                per_t{CtrlA2_t::c256, 256_ms},
                per_t{CtrlA2_t::c512, 512_ms},
                per_t{CtrlA2_t::c1k, 1024_ms},
                per_t{CtrlA2_t::c2k, 2048_ms},
                per_t{CtrlA2_t::c4k, 4096_ms},
                per_t{CtrlA2_t::c8k, 8192_ms},
            };
            
            ControlRegister<WatchDog, Meta::List<CtrlA1_t, CtrlA2_t>> ctrla;
            
            enum class Status_t: uint8_t {
                lock = (0x01 << 7),
                busy = (0x01 << 0),
            };
            FlagRegister<WatchDog, Status_t, ReadOnly> status;
            
            static inline constexpr uintptr_t address = 0x0100;
        };
       static_assert(sizeof(WatchDog) == 2);

    }
}
