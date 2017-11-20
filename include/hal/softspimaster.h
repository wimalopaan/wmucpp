/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "config.h"
#include "mcu/ports.h"
#include "mcu/avr/delay.h"
#include "util/bits.h"

// todo: BitMode (fuer Constant / Variable-Rate)

template<typename DataPin, typename ClockPin, typename CSPin = void, bool useDelay = false, typename T = std::byte>
class SoftSpiMaster final {
public:
    SoftSpiMaster() = delete;
    template<uint32_t Baud = 0>
    static void init() {
        static_assert(Baud == 0, "SoftSpiMaster doesn't use baud template parameter");
        DataPin::template dir<AVR::Output>();
        ClockPin::template dir<AVR::Output>();
        if constexpr(!std::is_same<CSPin, void>::value) {
            CSPin::template dir<AVR::Output>();
            CSPin::high();
        }
        ClockPin::low();
        DataPin::low();
    }

    static bool put(T value) {
        mData = value;
        if constexpr(!std::is_same<CSPin, void>::value) {
            CSPin::low();
            if constexpr(useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
        }
//        PulseBits<Util::numberOfBits<T>()>();

        for(uint8_t i = 0; i < Util::numberOfBits<T>(); ++i) {
            pulseNextBit();
        }

        if constexpr(!std::is_same<CSPin, void>::value) {
            if constexpr(useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
            CSPin::high();
        }
        DataPin::low();
        return true;
    }
    
    static inline void setData(std::byte data) {
        mData = data;
    }

    static inline void pulseNextBit() {
        if (Util::isSet<Util::MSB>(mData)) {
            DataPin::high();
        }
        else {
            DataPin::low();
        }
        if constexpr(useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
        ClockPin::high();
        mData <<= 1; // shift next bit
        if constexpr(useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
        ClockPin::low();
    }

private:
//    template<uint8_t Bits>
//    static inline void PulseBits() {
//        if (Util::isSet<Util::MSB>(mData)) {
//            DataPin::high();
//        }
//        else {
//            DataPin::low();
//        }
//        if constexpr(useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
//        ClockPin::high();
//        mData <<= 1; // shift next bit
//        if constexpr(useDelay) Util::delay(Config::SoftSpiMaster::pulseDelay);
//        ClockPin::low();
        
//        if constexpr((Bits - 1) > 0) {
//            PulseBits<Bits - 1>();
//        }
//    }
    inline static T mData{0};
};