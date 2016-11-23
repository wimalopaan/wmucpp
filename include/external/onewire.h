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

#include "config.h"
#include "mcu/ports.h"
#include "mcu/avr/util.h"
#include "mcu/avr/delay.h"
#include "util/disable.h"

namespace OneWire {
namespace detail {}

struct Normal {};
struct OverDrive{};

template<typename Mode>
struct Parameter;

template<>
struct Parameter<Normal> {
    static constexpr std::microseconds recovery = 10_us;
    static constexpr std::microseconds pre = 6_us;
    static constexpr std::microseconds zeroTotal = 60_us;
    static constexpr std::microseconds sampleAfterPre = 9_us;
    static constexpr std::microseconds reset = 480_us;
    static constexpr std::microseconds presenceAfterReset= 70_us;
};

template<>
struct Parameter<OverDrive> {
    static constexpr std::centimicroseconds pre{10};
    static constexpr std::centimicroseconds zeroTotal{75};
    static constexpr std::centimicroseconds sampleAfterPre{10};
    static constexpr std::centimicroseconds reset{700};
    static constexpr std::centimicroseconds presenceAfterReset{85};
};

template<typename Pin, typename Mode, bool InternalPullup = true>
class Master final {
public:
    Master() = delete;
    
    typedef Pin pin_type;
    typedef Mode mode_type;
    
    static void init() {
        Set<Pin>::output();
        Pin::high();
    }
    
    struct Synchronous {
        static bool reset() {
            bool presence = false;
            Pin::low();
            Set<Pin>::output();
            Util::delay(Parameter<Mode>::reset);       
            {
                Scoped<DisbaleInterrupt> di;
                Set<Pin>::input();
                if constexpr(InternalPullup) {
                    Pin::pullup();
                }
                Util::delay(Parameter<Mode>::presenceAfterReset);
                presence = !Pin::read(); // actice low
            }
            Util::delay(Parameter<Mode>::reset - Parameter<Mode>::presenceAfterReset);    
            return presence;
        }
        static void writeBit(bool bit) {
            Util::delay(Parameter<Mode>::recovery);
            Pin::low();
            Set<Pin>::output();
            {
                Scoped<DisbaleInterrupt> di;
                Util::delay(Parameter<Mode>::pre); 
                if (bit) {
                    Set<Pin>::input();
                    Pin::pullup();      
                }
                Util::delay(Parameter<Mode>::zeroTotal - Parameter<Mode>::pre);
            }
            Set<Pin>::input();
            Pin::pullup();      
        }    
        static bool readBit() {
            Util::delay(Parameter<Mode>::recovery);
            {
                Scoped<DisbaleInterrupt> di;
                Pin::low();
                Set<Pin>::output();
                Util::delay(Parameter<Mode>::pre); 
                Set<Pin>::input();
                Pin::pullup();      
                Util::delay(Parameter<Mode>::sampleAfterPre);
                return Pin::read();
            }
        }
        static uint8_t read() {
            uint8_t result = 0;
            for(uint8_t i = 0; i < 8; ++i) {
                result >>= 1;
                if (readBit()) {
                    result |= 0x80;
                }
            }
            return result;
        }
        static void write(uint8_t byte) {
            for(uint8_t i = 0; i < 8; ++i) {
                writeBit(byte & 0x01);
                byte >>= 1;
            }        
        }
        
    };
    
    struct Asynchronous {
        static void resetStart(){
            Pin::low();
            Set<Pin>::output();
        }
        static void resetStop(){
            Set<Pin>::input();
            if constexpr(InternalPullup) {
                Pin::pullup();
            }
        }
        static bool isPresence() {
            return !Pin::read();
        }
    };
    
    
    
};

}