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

#define NDEBUG

#include <stdint.h>
#include "std/type_traits"
#include "std/algorithm"
#include "std/utility"
#include "util/util.h"
#include "util/bits.h"
#include "util/static_container.h"


namespace AVR {

struct ReadWrite {};
struct ReadOnly{};

template<typename Component, typename BitType, typename ValueType = uint8_t>
struct ControlRegister {
    typedef Component component_type;
    typedef ValueType value_type;    
    typedef BitType bit_type;
    
    template<typename... Flags>
    void inline set(Flags... f) {
        static_assert((std::is_same<bit_type, Flags>::value && ... && true), "wrong flag type");
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        value_type v = (static_cast<value_type>(f) | ... | 0);
        value_type n = Util::numberOfOnes(v);        
        assert(n == sizeof... (Flags));
        hwRegister = v;
    }

    template<typename F, F... FF>
    void inline set(Util::static_container<F, FF...>) {
        set<FF...>();
    }
    
    template<BitType... Flags>
    void inline set() {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (static_cast<value_type>(Flags) | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        hwRegister = v;
    }

    template<BitType... Flags>
    void inline add() {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (static_cast<value_type>(Flags) | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        hwRegister |= v;
    }

    template<BitType... Flags>
    void inline clear() {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (static_cast<value_type>(Flags) | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        hwRegister &= ~v;
    }
    
    volatile value_type hwRegister;
};

template<typename Component, typename Mode = ReadWrite, typename ValueType = uint8_t>
struct DataRegister {
    typedef Component component_type;
    typedef ValueType value_type;    

    // SFINAE
    template<typename Dummy = void, 
             typename Dummy2 = typename std::enable_if<std::is_same<Mode, ReadWrite>::value, Dummy>::type>
    inline void operator=(value_type v) {
        hwRegister = v;
    }
    
    volatile value_type hwRegister;
};

}

namespace AVR {

template<typename Component>
constexpr inline Component* getBaseAddr() {
    return reinterpret_cast<Component*>(Component::address);
}

template<typename Component, int Number>
constexpr inline Component* getBaseAddr() {
    return reinterpret_cast<Component*>(Component::template Address<Number>::value);
}

template<typename Component, typename Letter>
constexpr inline Component* getBaseAddr() {
    return reinterpret_cast<Component* const>(Component::template Address<Letter>::value);
}

}

namespace AVR {

template<typename TCCR_TYPE>
struct PrescalerPair {
    typedef TCCR_TYPE  bits_type;
    typedef uint16_t scale_type;
    const TCCR_TYPE bits;
    const uint16_t scale;
};

struct ATTest final {
    struct Timer8Bit {
        DataRegister<Timer8Bit> ocrb;
        DataRegister<Timer8Bit> ocra;

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
}

namespace std {
template<>
struct enable_bitmask_operators<AVR::ATTest::Timer8Bit::TCCRA>{
    static const bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTest::Timer8Bit::TCCRB>{
    static const bool enable = true;
};
}

namespace AVR {

template<>
struct ATTest::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x10;
};



template<>
struct ATTest::Timer8Bit::PrescalerBits<0> {
    using tb = ATTest::Timer8Bit::TCCRB;
    static constexpr AVR::PrescalerPair<tb> values[] = {
        {tb::cs0 | tb::cs1, 1024},
        {tb::cs0 | tb::cs1, 256},
        {tb::cs0 | tb::cs1, 64},
        {tb::cs0 | tb::cs1, 8},
        {tb::cs0, 1},
        {static_cast<tb>(0), 0}
    };
};
template<>
struct ATTest::Timer8Bit::PrescalerBits<2> {
    using tb = ATTest::Timer8Bit::TCCRB;
    static constexpr AVR::PrescalerPair<tb> values[] = {
        {tb::cs0 | tb::cs1, 1024},
        {tb::cs0 | tb::cs1, 256},
        {tb::cs0 | tb::cs1, 64},
        {tb::cs0 | tb::cs1, 8},
        {tb::cs0, 1},
        {static_cast<tb>(0), 0}
    };
};

}

constexpr auto timer0 = AVR::getBaseAddr<AVR::ATTest::Timer8Bit, 0>;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"

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
#pragma GCC diagnostic pop

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {}
}
#endif
