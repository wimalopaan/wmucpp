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
    namespace Series1 {
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
            ControlRegister<Events, Strobe_t> asyncStrobe;
            ControlRegister<Events, Strobe_t> syncStrobe;
            
            enum class AsyncGenerator_t : uint8_t {
                off          = 0x00,
                ccl_lut0     = 0x01,
                ccl_lut1     = 0x02,
                ac0_out      = 0x03,
                tcd0_cmpBclr = 0x04,
                tcd0_cmpAset = 0x05,
                tcd0_cmpBset = 0x06,
                tcd0_progev  = 0x07,
                rtc_ovf      = 0x08,
                rtc_cmp      = 0x09,
                port_pin0    = 0x0a,
                pit_div8192  = 0x0a,
                port_pin1    = 0x0b,
                port_pin2    = 0x0c,
                port_pin3    = 0x0d,
                port_pin4    = 0x0e,
                port_pin5    = 0x0f,
                port_pin6    = 0x10,
                port_pin7    = 0x11,
                updi         = 0x12,           
                ac_out_x1    = 0x12,           
                ac_out_x2    = 0x13,           
                ac_out_x3    = 0x14
            };
            std::array<ControlRegister<Events, AsyncGenerator_t>, 4> async_channels;
        
            volatile const uint8_t reserved[4];
            
            enum class SyncGenerator_t : uint8_t {
                off          = 0x00,    
                tcb0         = 0x01,
                tca0_ovf_lunf= 0x02,
                tca0_hunf    = 0x03,
                tca0_cmp0    = 0x04,
                tca0_cmp1    = 0x05,
                tca0_cmp2    = 0x06,
                
                portc_pin0    = 0x07,
                portc_pin1    = 0x08,
                portc_pin2    = 0x09,
                portc_pin3    = 0x0a,
                portc_pin4    = 0x0b,
                portc_pin5    = 0x0c,

                porta_pin0    = 0x0d,
                porta_pin1    = 0x0e,
                porta_pin2    = 0x0f,
                porta_pin3    = 0x10,
                tcb1_x1       = 0x10,
                porta_pin4    = 0x11,
                porta_pin5    = 0x12,
                porta_pin6    = 0x13,
                porta_pin7    = 0x14,
                tcb1_x2       = 0x15
            };
            std::array<ControlRegister<Events, SyncGenerator_t>, 2> sync_channels;

            volatile const uint8_t reserved2[6];
            
            enum class AsyncChannel_t : uint8_t {
                off      = 0x00,
                syncch0  = 0x01,
                syncch1  = 0x02,
                asyncch0 = 0x03,
                asyncch1 = 0x04,
                asyncch2 = 0x05,
                asyncch3 = 0x06,
            };
            std::array<ControlRegister<Events, AsyncChannel_t>, 13> async_users;
            
            volatile const uint8_t reserved3[3];

            enum class SyncChannel_t : uint8_t {
                off      = 0x00,
                syncch0  = 0x01,
                snycch1  = 0x02,
            };
            std::array<ControlRegister<Events, SyncChannel_t>, 2> sync_users;

            static inline constexpr uintptr_t address = 0x0180;
        };
        static_assert(sizeof(Events) == 0x24);
    }
}
