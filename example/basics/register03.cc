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

//#define NDEBUG

#include <stdint.h>
#include "std/type_traits"
#include "std/algorithm"
#include "std/utility"
#include "util/util.h"
#include "util/static_container.h"

namespace AVR {

template<typename E>
struct EnumTraits;

struct ReadWrite {};
struct ReadOnly{};

enum class Empty {};

template<typename TCCR_TYPE>
struct PrescalerPair {
    typedef TCCR_TYPE  bits_type;
    typedef uint16_t scale_type;
    TCCR_TYPE  bits;
    uint16_t scale;
};

struct Prescaler_10Bit {
    enum class ScaleA: uint8_t {
        cs0         = (1 << CS00),
        cs1         = (1 << CS01),
        cs2         = (1 << CS02),
        cdiv1024    = _BV(CS12) |             _BV(CS10), 
        cdiv256     = _BV(CS12)                        ,
        cdiv64      =             _BV(CS11) | _BV(CS10),
        cdiv8       =             _BV(CS11)            ,
        cdiv1       =                         _BV(CS10),
        cdiv0       = 0,
    };
    enum class ScaleB: uint8_t {
        cs0         = (1 << CS00),
        cs1         = (1 << CS01),
        cs2         = (1 << CS02),
        cdiv1024 = _BV(CS02) | _BV(CS01) | _BV(CS00),
        cdiv256  = _BV(CS02) | _BV(CS01),
        cdiv128  = _BV(CS02)             | _BV(CS00),
        cdiv64   = _BV(CS02),
        cdiv32   =             _BV(CS01) | _BV(CS00),
        cdiv8    =             _BV(CS01),
        cdiv1    =                         _BV(CS00),
        cdiv0    = 0
    };
    static constexpr AVR::PrescalerPair<ScaleA> valuesA[] = {
        {ScaleA::cdiv1024, 1024},
        {ScaleA::cdiv256, 256},
        {ScaleA::cdiv64, 64},
        {ScaleA::cdiv8, 8},
        {ScaleA::cdiv1, 1},
        {ScaleA::cdiv0, 0}
    };
};

template<typename Register, typename BitType, uint8_t StartBit, uint8_t Span>
struct ControlRegisterPart final {
    typedef BitType bit_type;
    typedef typename Register::value_type value_type;    
//    static constexpr auto r = Register::hwRegister;
    
    constexpr explicit ControlRegisterPart(Register *r) {
        fullRegister = r;
    }

    template<typename... Flags>
    void inline set(Flags... f) {
//        static_assert((std::is_same<bit_type, Flags>::value && ... && true), "wrong flag type");
        static_assert(sizeof...(Flags) <= Span, "too much flags");
        static_assert(EnumTraits<BitType>::max < (1 << (Span + StartBit)), "wrong span for values");
//        value_type v = (static_cast<value_type>(f) | ... | 0);
        
        fullRegister->set(f...);
    }
    
private:
    static Register *fullRegister;
};
template<typename Register, typename BitType, uint8_t StartBit, uint8_t Span>
Register* ControlRegisterPart<Register, BitType, StartBit, Span>::fullRegister = nullptr;

template<typename Component, typename BitType = Empty, typename ValueType = uint8_t>
struct ControlRegister final {
    ControlRegister() = delete;
    
    template<typename, typename, uint8_t, uint8_t> friend class ControlRegisterPart;
    
    typedef Component component_type;
    typedef ValueType value_type;    
    typedef BitType bit_type;
    
    template<typename... Flags>
    void inline set(Flags... f) {
        static_assert((std::is_same<bit_type, Flags>::value && ... && true), "wrong flag type");
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        value_type v = (static_cast<value_type>(f) | ... | 0);
//        uint8_t n = Util::numberOfOnes(v);        
        assert(Util::numberOfOnes(v) == sizeof... (Flags));
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
private:    
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

template<template<uint8_t> class Component, int Number>
constexpr inline Component<Number>* getBaseAddr() {
    return reinterpret_cast<Component<Number>*>(Component<Number>::address);
}

template<typename Component, typename Letter>
constexpr inline Component* getBaseAddr() {
    return reinterpret_cast<Component* const>(Component::template Address<Letter>::value);
}

}

namespace AVR {

struct ATTest final {
    
    template<uint8_t N>
    struct Timer8BitMap;
    
    template<uint8_t N>
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
            wgm3 = (1 << WGM13),
            wgm2 = (1 << WGM12),
            icnc = (1 << ICNC1),
            ices = (1 << ICES1)
        };

        // union wuerde UB hervorrufen
        ControlRegister<Timer8Bit> tccrb;
        ControlRegisterPart<ControlRegister<Timer8Bit>, TCCRB, 3, 5> tccrb1{&tccrb};
////        ControlRegisterPart<ControlRegister<Timer8Bit>, Prescaler_10Bit::ScaleA, 0, 3> pre{tccrb};
//        ControlRegisterPart<ControlRegister<Timer8Bit>, typename Timer8BitMap<N>::scale, 0, 3> pre(&tccrb);
        
//        template<int N> struct Address;
//        template<int N> struct PrescalerBits;
//        struct Address;
        static constexpr uint8_t address = Timer8BitMap<N>::address;
        struct PrescalerBits;
    };
    struct PCInterrupts {
        volatile uint8_t pcmsk;
        template<int N> struct Address;
    };
};

template<>
struct ATTest::Timer8BitMap<0> {
    using scale = AVR::Prescaler_10Bit::ScaleA;
    static constexpr uint8_t address = 0x20;
};
template<>
struct ATTest::Timer8BitMap<2> {
    using scale = AVR::Prescaler_10Bit::ScaleB;
    static constexpr uint8_t address = 0x40;
};


template<>
struct ATTest::Timer8Bit<0>::PrescalerBits {
    static constexpr auto values = AVR::Prescaler_10Bit::valuesA;
};

struct EnumTraitsBase {
    template<typename T>
    static constexpr auto minimumEnumValue(const T& values) {
        typedef typename std::underlying_type<typename T::value_type>::type UT;
        auto min = static_cast<UT>(values[0]); 
        for(size_t i = 1; i < values.size; ++i) {
            auto v = static_cast<UT>(values[i]);
            if (v < min) {
                min = v;
            }
        }
        return min;
    }
    template<typename T>
    static constexpr auto maximumEnumValue(const T& values) {
        typedef typename std::underlying_type<typename T::value_type>::type UT;
        auto max = static_cast<UT>(values[0]); 
        for(size_t i = 1; i < values.size; ++i) {
            auto v = static_cast<UT>(values[i]);
            if (v > max) {
                max = v;
            }
        }
        return max;
    }
};

template<>
struct EnumTraits<Prescaler_10Bit::ScaleA> : public EnumTraitsBase {
    typedef Prescaler_10Bit::ScaleA value_type;
    
    static constexpr auto values = std::make_array(value_type::cs0, value_type::cs1, value_type::cs2);
    static constexpr auto size = values.size;
    
    static constexpr auto min = minimumEnumValue(values);
    static constexpr auto max = maximumEnumValue(values);
};
template<>
struct EnumTraits<Prescaler_10Bit::ScaleB> : public EnumTraitsBase {
    typedef Prescaler_10Bit::ScaleB value_type;
    
    static constexpr auto values = std::make_array(value_type::cs0, value_type::cs1, value_type::cs2);
    static constexpr auto size = values.size;
    
    static constexpr auto min = minimumEnumValue(values);
    static constexpr auto max = maximumEnumValue(values);
};

template<>
struct EnumTraits<ATTest::Timer8Bit<0>::TCCRB> : public EnumTraitsBase {
    typedef ATTest::Timer8Bit<0>::TCCRB value_type;
    
    static constexpr auto values = std::make_array(value_type::ices, value_type::icnc, value_type::wgm2, value_type::wgm3);
    static constexpr auto size = values.size;
    
    static constexpr auto min = minimumEnumValue(values);
    static constexpr auto max = maximumEnumValue(values);
};

//template<>
//struct ATTest::Timer8Bit<0>::Address {
//    static constexpr uint8_t value = 0x10;
//};

}

constexpr auto timer0 = AVR::getBaseAddr<AVR::ATTest::Timer8Bit, 0>;
//constexpr auto timer0 = AVR::getBaseAddr<AVR::ATTest::Timer8Bit<0>>;
constexpr auto timer2 = AVR::getBaseAddr<AVR::ATTest::Timer8Bit<2>>;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"

int main() {
    using ta = AVR::ATTest::Timer8Bit<0>::TCCRA;
    using tb = AVR::ATTest::Timer8Bit<0>::TCCRB;
    using preA = AVR::Prescaler_10Bit::ScaleA;
    using preB = AVR::Prescaler_10Bit::ScaleB;

    timer0()->tccra.set(ta::coma0, ta::coma1, ta::coma1);
//    timer0()->tccrb.set(ta::coma0); // wrong
//    timer0()->tccrb.set(tb::icnc);
//    timer0()->tccrb.set(clk10::cs0);
//    timer0()->tccrb.set<clk10::cs0, clk10::cs0>();

//    timer0()->pre.set(preA::cs0, preA::cs1);
//    timer2()->pre.set(preB::cs0, preB::cs1);
    
//    timer0()->clock.set(tb::ices);
    
//    timer0()->ocra = 100;
//    timer0()->ocrb = 100;
    
//    constexpr Util::static_container<ta, ta::coma0, ta::comb0> fs;
//    timer0()->tccra.set(fs);
    
//    constexpr auto fs2 = make_static_container<ta::coma0, ta::comb0>();
//    timer0()->tccra.set(fs2);
    
    while(true) {}
}
#pragma GCC diagnostic push

#ifndef NDEBUG
void assertFunction([[maybe_unused]] const PgmStringView&, [[maybe_unused]] const PgmStringView&, [[maybe_unused]] unsigned int) noexcept {
    while(true) {}
}
#endif
