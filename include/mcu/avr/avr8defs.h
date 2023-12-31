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

#if __has_include(<avr/io.h>)
# include <avr/io.h>
#endif
#if __has_include(<util/twi.h>)
# include <util/twi.h>
#endif

#include <array>

namespace AVR {
    
    template<typename BitsType>
    struct PrescalerPair {
        typedef BitsType  bits_type;
        typedef uint16_t scale_type;
        const BitsType  bits;
        const uint16_t scale;
    };
    
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 4> twiPrescalerBit = {
        PrescalerPair<CS>{CS::twps1 | CS::twps0, 64},
        PrescalerPair<CS>{CS::twps1           , 16},
        PrescalerPair<CS>{            CS::twps0,  4},
        PrescalerPair<CS>{static_cast<CS>(0) ,  1},
    };
    
    
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 6> prescalerValues10Bit = {
        PrescalerPair<CS>{CS::cs2          | CS::cs0, 1024},
        PrescalerPair<CS>{CS::cs2                   , 256},
        PrescalerPair<CS>{         CS::cs1 | CS::cs0, 64},
        PrescalerPair<CS>{         CS::cs1          , 8},
        PrescalerPair<CS>{                   CS::cs0, 1},
        PrescalerPair<CS>{static_cast<CS>(0)        , 0},
    };
    
    template<typename CS>
    constexpr CS csMask10Bit = {CS::cs2 | CS::cs1 | CS::cs0};
    
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 8> prescalerValues10BitExtended = {
        PrescalerPair<CS>{CS::cs2 | CS::cs1 | CS::cs0, 1024},
        PrescalerPair<CS>{CS::cs2 | CS::cs1          , 256},
        PrescalerPair<CS>{CS::cs2           | CS::cs0, 128},
        PrescalerPair<CS>{CS::cs2                    , 64},
        PrescalerPair<CS>{          CS::cs1 | CS::cs0, 32},
        PrescalerPair<CS>{          CS::cs1          , 8},
        PrescalerPair<CS>{                    CS::cs0, 1},
        PrescalerPair<CS>{static_cast<CS>(0)         , 0}
    };
    template<typename CS>
    constexpr std::array<PrescalerPair<CS>, 16> prescalerValues14Bit = {
        PrescalerPair<CS>{CS::cs3 | CS::cs2 | CS::cs1 | CS::cs0, 16384},
        PrescalerPair<CS>{CS::cs3 | CS::cs2 | CS::cs1          , 8192},
        PrescalerPair<CS>{CS::cs3 | CS::cs2 |           CS::cs0, 4096},
        PrescalerPair<CS>{CS::cs3 | CS::cs2                    , 2048},
        PrescalerPair<CS>{CS::cs3 |           CS::cs1 | CS::cs0, 1024},
        PrescalerPair<CS>{CS::cs3 |           CS::cs1          , 512},
        PrescalerPair<CS>{CS::cs3 |                     CS::cs0, 256},
        PrescalerPair<CS>{CS::cs3                              , 128},
        PrescalerPair<CS>{          CS::cs2 | CS::cs1 | CS::cs0, 64},
        PrescalerPair<CS>{          CS::cs2 | CS::cs1          , 32},
        PrescalerPair<CS>{          CS::cs2 |           CS::cs0, 16},
        PrescalerPair<CS>{          CS::cs2                    , 8},
        PrescalerPair<CS>{                    CS::cs1 | CS::cs0, 4},
        PrescalerPair<CS>{                    CS::cs1          , 2},
        PrescalerPair<CS>{                              CS::cs0, 1},
        PrescalerPair<CS>{static_cast<CS>(0)                   , 0}
    };
    
    template<typename CS>
    constexpr CS csMask14Bit = {CS::cs3 | CS::cs2 | CS::cs1 | CS::cs0};
    
    struct A {
        static constexpr char letter = 'A';
    };
    struct B {
        static constexpr char letter = 'B';
    };
    struct C {
        static constexpr char letter = 'C';
    };
    struct D {
        static constexpr char letter = 'D';
    };
    struct E {
        static constexpr char letter = 'E';
    };
    
} //!AVR
