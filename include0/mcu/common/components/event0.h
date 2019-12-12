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

namespace AVR {
    namespace Series0 {
        struct Events {
            enum class Strobe_t : uint8_t {
                ch0 = 0x01,
                ch1 = 0x02,
                ch2 = 0x04,
                ch3 = 0x08,
                ch4 = 0x10,
                ch5 = 0x20,
                ch6 = 0x40,
                ch7 = 0x80,
            };
            ControlRegister<Events, Strobe_t> strobe;

            volatile uint8_t reserved[15];
            
            enum class Generator_t : uint8_t {
                updi       = 0x01,
                rtc_ovf    = 0x06,
                rtc_cmp    = 0x07,
                rtc_pit0   = 0x08,
                rtc_pit1   = 0x09,
                rtc_pit2   = 0x0a,
                rtc_pit3   = 0x0b,
                ccl_lut0   = 0x10,
                ccl_lut1   = 0x11,
                ccl_lut2   = 0x12,
                ccl_lut3   = 0x13,
                ac0_out    = 0x20,
                adc0_rdy   = 0x24,
                port0_pin0 = 0x40,
                port0_pin1 = 0x41,
                port0_pin2 = 0x42,
                port0_pin3 = 0x43,
                port0_pin4 = 0x44,
                port0_pin5 = 0x45,
                port0_pin6 = 0x46,
                port0_pin7 = 0x47,
                port1_pin0 = 0x48,
                port1_pin1 = 0x49,
                port1_pin2 = 0x4a,
                port1_pin3 = 0x4b,
                port1_pin4 = 0x4c,
                port1_pin5 = 0x4d,
                port1_pin6 = 0x4e,
                port1_pin7 = 0x4f,
                usart0_xck = 0x60,           
                usart1_xck = 0x61,           
                usart2_xck = 0x62,           
                usart3_xck = 0x63,           
                spi0_sck   = 0x68,           
                tca0_ovf   = 0x80,           
                tca0_hunf  = 0x81,           
                tca0_cmp0  = 0x84,           
                tca0_cmp1  = 0x85,           
                tca0_cmp2  = 0x86,           
                tcab0_capt = 0xa0,           
                tcab1_capt = 0xa1,           
                tcab2_capt = 0xa2,           
                tcab3_capt = 0xa3,           
            };
            std::array<ControlRegister<Events, Generator_t>, 8> channels;
        
            volatile uint8_t reserved2[8];
            
            enum class Channel_t : uint8_t {
                ch0 = 0x00,
                ch1 = 0x01,
                ch2 = 0x02,
                ch3 = 0x03,
                ch4 = 0x04,
                ch5 = 0x05,
                ch6 = 0x06,
                ch7 = 0x07,
            };
            std::array<ControlRegister<Events, Channel_t>, 24> users;
            
            static inline constexpr uintptr_t address = 0x0180;
            
        };
        
        static_assert(sizeof(Events) == 56);
    }
}
