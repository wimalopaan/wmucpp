/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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
#include "mcu/ports.h"
#include "util/bits.h"
#include "std/algorithm.h"

// Bit Fields sind für Register ungeeeignet, da die Anordnung im Speicher nicht garantiert ist.
// http://en.cppreference.com/w/cpp/language/bit_field

namespace AVR {

template<typename Register, int N, typename Register::value_type v>
struct McuRegisterFlag {
    static constexpr typename Register::value_type value = (1 << v);
    static constexpr int number = N;
    typedef typename Register::value_type value_type;
    typedef Register register_type;
    typedef McuRegisterFlag type;
//    constexpr operator value_type() const noexcept { return value; }
//    constexpr value_type operator()() const noexcept { return value; } 
};

template<typename Register> 
struct McuRegisterFlags {
    typedef Register register_type;
    typedef typename Register::value_type value_type;
};

template<typename Component, uint8_t Index, typename ValueType = uint8_t>
struct McuRegister {
    static constexpr uint8_t index = Index;
    typedef Component component_type;
    typedef ValueType value_type;    
    typedef McuRegister register_type;
    
    template<typename... Flags>
    void set(Flags... f) {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (f.value | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        static_assert((std::is_same<register_type, typename Flags::register_type>::value && ... && true), "must use same register");

        typedef typename ::Util::nth_element<0, Flags...> first_flag_type;
        constexpr auto first_number = first_flag_type::number;
        
        //constexpr bool x = ((Flags::number == first_number) && ... && true);
        
        static_assert(true, "");
        reg = v;
    }
    
    volatile value_type reg;
};


struct ATTest final {
    struct Timer8Bit {
        using tccra_t = McuRegister<Timer8Bit, 2>;

        template<int N> struct TccraFlags;

        volatile uint8_t ocrb;
        volatile uint8_t ocra;
        tccra_t tccra;
        
        volatile uint8_t padding[0x32 - 0x2A - 1];
        volatile uint8_t tcnt;
        volatile uint8_t tccrb;
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
    static constexpr uint8_t value = 0x48;
};

template<>
struct ATTest::Timer8Bit::TccraFlags<0> : public McuRegisterFlags<tccra_t> {
    static constexpr McuRegisterFlag<tccra_t, 0, COM1A0> com1a{};
    static constexpr McuRegisterFlag<tccra_t, 0, COM1B0> com1b{};
};

template<>
struct ATTest::Timer8Bit::TccraFlags<1> : public McuRegisterFlags<tccra_t> {
    static constexpr McuRegisterFlag<tccra_t, 1, COM1A1> com1a{};
    static constexpr McuRegisterFlag<tccra_t, 1, COM1B1> com1b{};
};

}

constexpr auto timer0 = AVR::getBaseAddr<AVR::ATTest::Timer8Bit, 0>;

int main() {
    using t0flags = AVR::ATTest::Timer8Bit::TccraFlags<0>;
    using t1flags = AVR::ATTest::Timer8Bit::TccraFlags<1>;
    
    timer0()->tccra.set(t0flags::com1a, t0flags::com1b);
    timer0()->tccra.set(t1flags::com1a, t1flags::com1b); // ok, all timers are structural equal
}
