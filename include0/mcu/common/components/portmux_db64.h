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
    namespace SeriesDb {
        struct Portmux {
            enum class EvRoute_t : uint8_t {
                A_alt1 = PORTMUX_EVOUTA_bm,
                C_alt1 = PORTMUX_EVOUTC_bm,
                D_alt1 = PORTMUX_EVOUTD_bm,
                F_alt1 = PORTMUX_EVOUTF_bm,
                G_alt1 = PORTMUX_EVOUTG_bm,
            };
            ControlRegister<Portmux, EvRoute_t> evsysroutea;

            enum class CclRoute_t : uint8_t {
                lut0_alt1 = PORTMUX_LUT0_bm,
                lut1_alt1 = PORTMUX_LUT1_bm,
                lut2_alt1 = PORTMUX_LUT2_bm,
                lut3_alt1 = PORTMUX_LUT3_bm,
                lut4_alt1 = PORTMUX_LUT4_bm,
                lut5_alt1 = PORTMUX_LUT5_bm,
            };
            ControlRegister<Portmux, CclRoute_t> cclroutea;
            
            enum class UsartRouteA_t : uint8_t {
                usart0_alt1 = PORTMUX_USART0_0_bm,
                usart0_none = PORTMUX_USART0_0_bm | PORTMUX_USART0_1_bm ,
                usart1_alt1 = PORTMUX_USART1_0_bm,
                usart1_none = PORTMUX_USART1_0_bm | PORTMUX_USART1_1_bm ,
                usart2_alt1 = PORTMUX_USART2_0_bm,
                usart2_none = PORTMUX_USART2_0_bm | PORTMUX_USART2_1_bm ,
                usart3_alt1 = PORTMUX_USART3_0_bm,
                usart3_none = PORTMUX_USART3_0_bm | PORTMUX_USART3_1_bm ,
            };
            ControlRegister<Portmux, UsartRouteA_t> usartroutea;

            enum class UsartRouteB_t : uint8_t {
                usart4_alt1 = PORTMUX_USART4_0_bm,
                usart4_none = PORTMUX_USART4_0_bm | PORTMUX_USART4_1_bm ,
                usart5_alt1 = PORTMUX_USART5_0_bm,
                usart5_none = PORTMUX_USART5_0_bm | PORTMUX_USART5_1_bm ,
            };
            ControlRegister<Portmux, UsartRouteB_t> usartrouteb;
            
            enum class SpiRoute_t : uint8_t {
                spi0_alt1 = PORTMUX_SPI0_0_bm,
                spi0_alt2 = PORTMUX_SPI0_1_bm,
                spi0_none = PORTMUX_SPI0_0_bm | PORTMUX_SPI0_1_bm,
                spi1_alt1 = PORTMUX_SPI1_0_bm,
                spi1_alt2 = PORTMUX_SPI1_1_bm,
                spi1_none = PORTMUX_SPI1_0_bm | PORTMUX_SPI1_1_bm,
            };
            ControlRegister<Portmux, SpiRoute_t> spiroutea;

            enum class TwiRoute_t : uint8_t {
                twi0_alt1 = PORTMUX_TWI0_0_bm,
                twi0_alt2 = PORTMUX_TWI0_1_bm,
                twi0_none = PORTMUX_TWI0_0_bm | PORTMUX_TWI0_1_bm,
                twi1_alt1 = PORTMUX_TWI1_0_bm,
                twi1_alt2 = PORTMUX_TWI1_1_bm,
                twi1_none = PORTMUX_TWI1_0_bm | PORTMUX_TWI1_1_bm,
            };
            ControlRegister<Portmux, TwiRoute_t> twiroutea;
            
            enum class Tca0Route_t : uint8_t {
                onA = 0x00,
                onB = 0x01,
                onC = 0x02,
                onD = 0x03,
                onE = 0x04,
                onF = 0x05,
                onG = 0x06,
            };
            enum class Tca1Route_t : uint8_t {
                onB = 0x01 << 3,
                onC = 0x01 << 3,
                onE = 0x02 << 3,
                onG = 0x03 << 3,
            };
            ControlRegister<Portmux, Meta::List<Tca0Route_t, Tca1Route_t>> tcaroutea;

            enum class TcbRoute_t : uint8_t {
                tcb0_alt1 = PORTMUX_TCB0_bm,
                tcb1_alt1 = PORTMUX_TCB1_bm,
                tcb2_alt1 = PORTMUX_TCB2_bm,
                tcb3_alt1 = PORTMUX_TCB3_bm,
                tcb4_alt1 = PORTMUX_TCB4_bm,
            };
            ControlRegister<Portmux, TcbRoute_t> tcbroutea;

            enum class TcdRoute_t : uint8_t {
                def  = 0x00,
                alt1 = 0x01,
                alt2 = 0x02,
                alt3 = 0x03
            };
            ControlRegister<Portmux, TcdRoute_t> tcdroutea;
            
            enum class AcRoute_t : uint8_t {
            };
            ControlRegister<Portmux, AcRoute_t> acroutea;

            enum class ZcdRoute_t : uint8_t {
            };
            ControlRegister<Portmux, ZcdRoute_t> zcdroutea;

            static inline constexpr uintptr_t address = 0x05e0;
        };
        static_assert(sizeof(Portmux) == (0x0a + 1));
    }
}
