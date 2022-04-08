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
        struct Portmux {
            enum class EvRoute_t : uint8_t {
                A_alt1 = PORTMUX_EVOUT0_bm,
                B_alt1 = PORTMUX_EVOUT1_bm,
                C_alt1 = PORTMUX_EVOUT2_bm,
                D_alt1 = PORTMUX_EVOUT3_bm,
                E_alt1 = PORTMUX_EVOUT4_bm,
                F_alt1 = PORTMUX_EVOUT5_bm,
            };
            ControlRegister<Portmux, EvRoute_t> evsysroutea;

            enum class CclRoute_t : uint8_t {
                lut0_alt1 = PORTMUX_LUT0_bm,
                lut1_alt1 = PORTMUX_LUT1_bm,
                lut2_alt1 = PORTMUX_LUT2_bm,
                lut3_alt1 = PORTMUX_LUT3_bm,
            };
            ControlRegister<Portmux, CclRoute_t> cclroutea;
            
            enum class UsartRoute_t : uint8_t {
                usart0_alt1 = PORTMUX_USART00_bm,
                usart0_none = PORTMUX_USART00_bm | PORTMUX_USART01_bm ,
                usart1_alt1 = PORTMUX_USART10_bm,
                usart1_none = PORTMUX_USART10_bm | PORTMUX_USART11_bm ,
                usart2_alt1 = PORTMUX_USART20_bm,
                usart2_none = PORTMUX_USART20_bm | PORTMUX_USART21_bm ,
                usart3_alt1 = PORTMUX_USART30_bm,
                usart3_none = PORTMUX_USART30_bm | PORTMUX_USART31_bm ,
            };
            ControlRegister<Portmux, UsartRoute_t> usartroutea;

            enum class TwiSpiRoute_t : uint8_t {
                spi0_alt1 = PORTMUX_SPI00_bm,
                spi0_alt2 = PORTMUX_SPI01_bm,
                spi0_none = PORTMUX_SPI00_bm | PORTMUX_SPI01_bm,
                twi0_alt1 = PORTMUX_TWI00_bm,
                twi0_alt2 = PORTMUX_TWI01_bm,
                twi0_none = PORTMUX_TWI00_bm | PORTMUX_TWI01_bm,
            };
            ControlRegister<Portmux, TwiSpiRoute_t> twispiroutea;

            enum class TcaRoute_t : uint8_t {
                onA = 0x00,
                onB = 0x01,
                onC = 0x02,
                onD = 0x03,
                onE = 0x04,
                onF = 0x05,
            };
            ControlRegister<Portmux, TcaRoute_t> tcaroutea;

            enum class TcbRoute_t : uint8_t {
                tcb0_alt1 = PORTMUX_TCB0_bm,
                tcb1_alt1 = PORTMUX_TCB1_bm,
                tcb2_alt1 = PORTMUX_TCB2_bm,
                tcb3_alt1 = PORTMUX_TCB3_bm,
            };
            ControlRegister<Portmux, TcbRoute_t> tcbroutea;
            
            static inline constexpr uintptr_t address = 0x05e0;
        };

    }
}
