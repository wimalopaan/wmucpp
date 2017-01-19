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

#include "mcu/avr8.h"
#include "mcu/register.h"
#include "mcu/ports.h"
#include "util/bits.h"
#include "std/algorithm.h"


// Bit Fields sind für Register ungeeeignet, da die Anordnung im Speicher nicht garantiert ist.
// http://en.cppreference.com/w/cpp/language/bit_field

namespace AVR {

struct ATTest final {
    struct Timer8Bit {
        DataRegister<Timer8Bit> ocrb;
        DataRegister<Timer8Bit> ocra;
//        McuDataRegister<Timer8Bit, ReadOnly> ocra;

        enum class TCCRA : uint8_t {
            coma0 = (1 << COM0A0),
            coma1 = (1 << COM0A1),
            comb0 = (1 << COM0B0),
            comb1 = (1 << COM0B1)
        };
        
        ControlRegister<Timer8Bit, TCCRA> tccra;
        
        volatile uint8_t padding[0x32 - 0x2A - 1];
        
        DataRegister<Timer8Bit> tcnt;

        enum class TCCRB : uint8_t {
            cs0 = (1 << CS10),
            cs1 = (1 << CS11),
            cs2 = (1 << CS12),
            wgm3 = (1 << WGM13),
            wgm2 = (1 << WGM12),
            icnc = (1 << ICNC1),
            ices = (1 << ICES1)
        };
        
        ControlRegister<Timer8Bit, TCCRB> tccrb;
        
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };
    struct PCInterrupts {
        volatile uint8_t pcmsk;
        template<int N> struct Address;
    };
};
template<>
struct ATTest::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x10;
};

}

constexpr auto timer0 = AVR::getBaseAddr<AVR::ATTest::Timer8Bit, 0>;

//template<typename T, typename... Ts>
//constexpr auto make_static_container() {
//    return Util::static_container<T, t0, ts...>{};
//}

int main() {
    using ta = AVR::ATTest::Timer8Bit::TCCRA;
    using tb = AVR::ATTest::Timer8Bit::TCCRB;

//    timer0()->tccra.set(ta::coma0, ta::coma1, ta::coma1);
//    timer0()->tccrb.set(ta::coma0); // wrong
//    timer0()->tccrb.set(tb::cs0);
//    timer0()->tccrb.set<tb::cs0, tb::cs0>();
  
//    timer0()->ocra = 100;
//    timer0()->ocrb = 100;
    
    constexpr Util::static_container<ta, ta::coma0, ta::comb0> fs;
    timer0()->tccra.set(fs);
    
//    constexpr auto fs2 = make_static_container<ta::coma0, ta::comb0>();
//    timer0()->tccra.set(fs2);
    
    while(true) {}
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif
