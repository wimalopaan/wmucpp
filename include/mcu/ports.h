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

#if __has_include(<avr/io.h>)
# include <avr/io.h>
#endif

#if __has_include(<avr/pgmspace.h>)
# include <avr/pgmspace.h>
#endif

#include "mcu/avr8.h"
#include "std/traits.h"
#include "util/algorithm.h"

namespace AVR {

struct Output final {
    Output() = delete;
    template<typename Port, uint8_t mask>
    static inline void set() {
        Port::dir() |= mask;
    }
};
struct Input final {
    Input() = delete;
    template<typename Port, uint8_t mask>
    static inline void set() {
        Port::dir() &= ~mask;
    }
};

template<typename MCUPort, typename Name>
struct Port final {
    typedef MCUPort mcuport_type;
    typedef Name name_type;
    Port() = delete;
    static inline void set(uint8_t v) {
        getBaseAddr<MCUPort, Name>()->out = v;
    }
    template<uint8_t V>
    static inline void set() {
        getBaseAddr<MCUPort, Name>()->out = V;
    }
    static inline volatile uint8_t& get() {
        return getBaseAddr<MCUPort, Name>()->out;
    }
    static inline void dir(uint8_t v) {
        getBaseAddr<MCUPort, Name>()->ddr = v;
    }
    template<uint8_t V>
    static inline void dir() {
        getBaseAddr<MCUPort, Name>()->ddr = V;
    }
    static inline volatile uint8_t& dir() {
        return getBaseAddr<MCUPort, Name>()->ddr;
    }
    static inline volatile uint8_t& read() {
        return getBaseAddr<MCUPort, Name>()->in;
    }
    static inline constexpr uintptr_t address() {
        return reinterpret_cast<uintptr_t>(&getBaseAddr<MCUPort, Name>()->out);
    }
};

template<typename... Pins>
class PinSet final {
public:
    static_assert(sizeof... (Pins) <= 8, "too much pins in PinSet");
    
    static constexpr uint8_t size = sizeof...(Pins);
    static constexpr uint8_t pinNumbers[] = {Pins::number...};
    static constexpr uint8_t pinMasks[] = {Pins::pinMask...};
    static constexpr uint8_t setMask = (Pins::pinMask | ... | 0);
    
    static constexpr auto calculatePatterns = [](){
        constexpr uint16_t numberOfPatterns = (1 << size);
        std::array<uint8_t, numberOfPatterns> data;
        for(uint8_t value = 0; value < numberOfPatterns; ++value) {
            uint8_t pattern = 0;
            uint8_t vv = value;
            for(uint8_t bit = 0; bit < size; ++bit) {
                if (vv & 0x01) {
                    pattern |= pinMasks[bit];
                }
                vv >>= 1;
            }
            data[value] = pattern;
        }
        return data;
    };
    
    // initialisiert valueBits mit den Bitmustern für die (ggf nicht benachbarten) Pins
    static constexpr auto valueBitsPGM PROGMEM = calculatePatterns();
    static constexpr auto valueBits = calculatePatterns();
    
    typedef typename ::Util::nth_element<0, Pins...>::port port_type;
    static_assert((std::is_same<port_type, typename Pins::port>::value && ... && true), "must use same port");
    
    static inline void allOn() {
        port_type::get() |= setMask;
    }
    static constexpr auto& allPullup = allOn;
    static inline void allOff() {
        port_type::get() &= ~setMask;
    }
    static inline uint8_t read() {
        return port_type::read() & setMask;
    }
    template<typename... PP>
    static inline void on() {
        constexpr uint8_t invertedMask = ~setMask;
        constexpr uint8_t mask = (PP::pinMask | ... | 0);
        static_assert(((mask & invertedMask) == 0), "Pin not in PinSet");
        port_type::get() |= mask;
    }    
    static constexpr auto& pullup = on;
    template<typename... PP>
    static inline void off() {
        constexpr uint8_t invertedMask = ~setMask;
        constexpr uint8_t mask = (PP::pinMask | ... | 0);
        static_assert(((mask & invertedMask) == 0), "Pin not in PinSet");
        port_type::get() &= ~mask;
    }  
    template<typename Dir>
    static inline void dir() {
        (Pins::template dir<Dir>(),...);
    }
    static inline void set(uint8_t v) {
        constexpr uint8_t value_mask = (1 << size) - 1;
        assert((v & ~value_mask) == 0);
        port_type::get() = (port_type::get() & ~setMask) | pgm_read_byte(&valueBitsPGM[v]); 
    }
    template<uint8_t V>
    static inline void set() {
        constexpr uint8_t value_mask = (1 << size) - 1;
        static_assert((V & ~value_mask) == 0, "wrong value V");
        port_type::get() = (port_type::get() & ~setMask) | valueBits[V]; 
    }
private:
    PinSet() = delete;
};

template<typename Port, uint8_t PinNumber>
struct Pin final {
    static_assert(PinNumber < 8, "wrong pin number");
    Pin() = delete;
    typedef Port port;
    static constexpr uint8_t number = PinNumber;
    static constexpr uint8_t pinMask = (1 << PinNumber);
    static inline void on() {
        Port::get() |= pinMask;
    }
    static constexpr auto& high = on;
    static constexpr auto& pullup = on;
    static inline void off() {
        Port::get() &= ~pinMask;
    }
    static constexpr auto& low = off;
    static inline void toggle() {
        Port::get() ^= pinMask;
    }
    template<typename Dir>
    static inline void dir() {
        Dir::template set<Port, pinMask>();
    }
    static inline bool read() {
        return Port::read() & pinMask;
    }
    static constexpr auto& isHigh = read;
};

struct NoPin final {
    NoPin() = delete;
    static constexpr uint8_t number = 0;
    static constexpr uint8_t pinMask = 0x01;
    static void on() {}
    static constexpr auto& high = on;
    static void off() {}
    static constexpr auto& low = off;
    static void toggle() {}
    template<typename Dir>
    static void dir() {}
    static bool read() {return false;}
};

struct ActiveLow {
    template<typename Pin>
    static void activate() {
        Pin::low();
    }
    template<typename Pin>
    static void inactivate() {
        Pin::high();
    }
};
struct ActiveHighl {
    template<typename Pin>
    static void activate() {
        Pin::high();
    }
    template<typename Pin>
    static void inactivate() {
        Pin::low();
    }    
};

template<typename Pin, typename Mode>
struct ScopedPin {
    ScopedPin() {
        Mode::template activate<Pin>();
    }
    ~ScopedPin() {
        Mode::template inactivate<Pin>();
    }
};

}

template<typename Pin>
struct Set {
    static void input() {
        Pin::template dir<AVR::Input>();
    }
    static void output() {
        Pin::template dir<AVR::Output>();
    }
};

