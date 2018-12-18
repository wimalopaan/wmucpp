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

//#include "mcu/avr8.h"
//#include "mcu/concepts.h"
//#include "container/pgmarray.h"
//#include "util/disable.h"
//#include "util/util.h"
//#include "util/meta.h"
//#include "util/types.h"
//#include "util/type_traits.h"

#include <cstddef>
#include <chrono>

#include <etl/types.h>

#include "../pgm/pgmarray.h"

namespace AVR {
    struct UsePgmTable{};
    
    struct Output final {
        Output() = delete;
        template<typename Port, std::byte mask>
        static inline void set() {
            Port::dir() |= mask;
        }
        template<typename Port>
        static inline void set() {
            Port::dir() = std::byte{0xff};
        }
    };
    struct Input final {
        Input() = delete;
        template<typename Port, std::byte mask>
        static inline void set() {
            Port::dir() &= ~mask;
        }
        template<typename Port>
        static inline void set() {
            Port::dir() = std::byte{0};
        }
    };
    
    template<typename MCUPort, AVR::Concepts::Letter Name>
    struct Port final {
        typedef MCUPort mcuport_type;
        typedef Name name_type;
        Port() = delete;
        static inline void set(std::byte v) {
            *getBaseAddr<MCUPort, Name>()->out = v;
        }
        template<std::byte V>
        static inline void set() {
            *getBaseAddr<MCUPort, Name>()->out = V;
        }
        static inline volatile std::byte& get() {
            return *getBaseAddr<MCUPort, Name>()->out;
        }
        static inline void dir(std::byte v) {
            *getBaseAddr<MCUPort, Name>()->ddr = v;
        }
        template<uint8_t V>
        static inline void dir() {
            *getBaseAddr<MCUPort, Name>()->ddr = V;
        }
        static inline volatile std::byte& dir() {
            return *getBaseAddr<MCUPort, Name>()->ddr;
        }
        static inline std::byte read() {
            return *getBaseAddr<MCUPort, Name>()->in;
        }
        template<uint8_t Bit>
        static inline void toggle() { 
            static_assert(AVR::isSBICBICapable<MCUPort, Name>(), "Port not sbi/sbi capable");
            *getBaseAddr<MCUPort, Name>()->in |= std::byte{(1 << Bit)}; // AVR specific toggle mechanism: write "1" to PortInputRegister
        }
        static inline void toggle(std::byte v) {
            *getBaseAddr<MCUPort, Name>()->in = v;
        }
        static inline constexpr uintptr_t address() {
            return reinterpret_cast<uintptr_t>(&getBaseAddr<MCUPort, Name>()->out);
        }
    };
    
    namespace detail::PinSet {
        
        template<typename... Pins>
        struct Generator {
            static_assert(sizeof...(Pins) <= 8);
            static inline constexpr uint8_t size = sizeof...(Pins);
            static constexpr std::byte pinMasks[] = {Pins::pinMask...};
            inline constexpr auto operator()() {
                constexpr uint16_t numberOfPatterns = (1 << size);
                std::array<std::byte, numberOfPatterns> data;
                for(uint16_t value = 0; value < numberOfPatterns; ++value) {
                    std::byte pattern{0};
                    std::byte vv{(uint8_t)value};
                    for(uint8_t bit = 0; bit < size; ++bit) {
                        if (std::any(vv & std::byte{0x01})) {
                            pattern |= pinMasks[bit];
                        }
                        vv >>= 1;
                    }
                    data[value] = pattern;
                }
                return data;
            }            
        };
        template<typename... Pins>
        struct GeneratorReverse {
            static_assert(sizeof...(Pins) <= 8);
            inline static constexpr std::byte mask = (Pins::mask | ...);
            inline constexpr auto operator()() {
                constexpr auto forward_patterns = Generator<Pins...>();
                constexpr uint16_t numberOfPatterns = (1u << etl::numberOfBits<std::byte>());
                std::array<std::byte, numberOfPatterns> data;
                for(uint16_t value = 0; value < numberOfPatterns; ++value) {
                    std::byte masked_value = std::byte(value) & mask;
                    for(uint16_t rv = 0; rv < forward_patterns.size; ++rv) {
                        if (forward_patterns[rv] == masked_value) {
                            data[value] = rv;
                            break;
                        }
                    }
                }
                return data;
            }            
        };
        
        template<bool usePgm, typename... Pins>
        struct Table;
        
        template<typename... Pins>
        struct Table<false, Pins...> {
            static inline constexpr uint8_t size = sizeof...(Pins);
            using generator_type = Generator<Pins...>;
            static inline constexpr auto valueBits = Generator<Pins...>()();
        };
        template<typename... Pins>
        struct Table<true, Pins...> {
            static inline constexpr uint8_t size = sizeof...(Pins);
            using generator_type = Generator<Pins...>;
            using t = typename ::Util::Pgm::Converter<generator_type>::pgm_type;
            static inline constexpr auto valueBits = t{};
        };
    }

    template<typename P1, typename... PPs>
    class PinSet final {
    public:
        static constexpr bool p1IsPin = !std::is_same<P1, UsePgmTable>::value;
        static constexpr bool usePgm = !p1IsPin;
        
        using pinlist = typename std::conditional<p1IsPin, Meta::List<P1, PPs...>, Meta::List<PPs...>>::type;
        
        using table_type = typename std::conditional<p1IsPin, detail::PinSet::Table<usePgm, P1, PPs...>, detail::PinSet::Table<usePgm, PPs...>>::type;
        static inline constexpr table_type table{};
        
        using gen_type = typename table_type::generator_type;
        
        static_assert(Meta::size<pinlist>::value <= 8, "too much pins in PinSet");
        static constexpr uint8_t size = Meta::size<pinlist>::value;
        static constexpr auto pinMasks = gen_type::pinMasks;
        static constexpr std::byte setMask = []{
            if constexpr(p1IsPin) {
                return P1::pinMask | (PPs::pinMask | ... | std::byte{0});
            }
            else {
                return (PPs::pinMask | ... | std::byte{0});
            }
        }();
        
        static_assert(Meta::is_set<pinlist>::value, "must use different pins in set"); // all Pins are different
        
        template<typename T> using port_from = typename T::port;
        using portlist = Meta::transform<port_from, pinlist>;
        typedef typename Meta::nth_element<0, portlist> port_type;
        static_assert(Meta::all_same<Meta::nth_element<0, portlist>, portlist>::value, "must use same port");
        
        static inline void allOn() { // race-free 
            auto v = (port_type::get() & setMask) ^ setMask;
            port_type::toggle(v);
        }
        template<typename I = etl::RestoreState>
        static inline void allPullup() {
            etl::Scoped<etl::DisbaleInterrupt<I>> di;
            port_type::get() |= setMask;
        }
        static inline void allOff() { // race-free
            auto v = port_type::get() & setMask;
            port_type::toggle(v);
        }
        static inline std::byte read() {
            return port_type::read() & setMask;
        }
        template<typename... PP>
        static inline void on() {
            static_assert(Meta::containsAll<pinlist, PP...>::value, "Pin not in PinSet");
            constexpr std::byte mask = (PP::pinMask | ... | std::byte{0});
            if constexpr(sizeof...(PP) == 1) {
                port_type::get() |= mask; // single-bit set -> sbi-instruction
            }
            else {
                auto v = (port_type::get() & mask) ^ mask; // race-free
                port_type::toggle(v);
            }
        }    
        template<typename... PP>
        static inline void pullup() {
            static_assert(Meta::containsAll<pinlist, PP...>::value, "Pin not in PinSet");
            constexpr std::byte mask = (PP::pinMask | ... | std::byte{0});
            etl::Scoped<etl::DisbaleInterrupt<etl::RestoreState>> di;
            port_type::get() |= mask;
        }
        template<typename... PP>
        static inline void off() {
            static_assert(Meta::containsAll<pinlist, PP...>::value, "Pin not in PinSet");
            constexpr std::byte mask = (PP::pinMask | ... | std::byte{0});
            if constexpr(sizeof...(PP) == 1) {
                port_type::get() &= ~mask;
            }
            else {
                auto v = port_type::get() & mask;
                port_type::toggle(v);
            }
        }  
        template<typename Dir>
        static inline void dir() {
            if constexpr(p1IsPin) {
                P1::template dir<Dir>();
                (PPs::template dir<Dir>(),...);
            }
            else {
                (PPs::template dir<Dir>(),...);
            }
        }
        // interprets value as bits corresponding to the (not adjacent) Pins
        // Pin1 = Pin<PortB,7>
        // Pin0 = Pin<PortB,1>
        // 0b00 -> Pin1(0) Pin0(0)
        // 0b01 -> Pin1(0) Pin0(1)
        // 0b10 -> Pin1(1) Pin0(0)
        // 0b11 -> Pin1(1) Pin0(1)
#if 0
        static inline void set(std::byte value) {
            assert(std::to_integer<uint8_t>(value) < valueBits.size);
            auto v = (port_type::get() & setMask) ^ valueBits[std::to_integer<uint8_t>(value)];
            port_type::toggle(v);
        }
#endif
        typedef etl::bitsN_t<size> bits_type;
        static inline void set(bits_type value) {
            auto v = (port_type::get() & setMask) ^ table.valueBits[static_cast<typename bits_type::value_type>(value)]; // <> race-free for non-overlapping PinSets
            port_type::toggle(v);
        }
        static inline void set(etl::uintN_t<size> value) {
            static_assert(std::numeric_limits<etl::uintN_t<size>>::max() < table.valueBits.size);
            auto v = (port_type::get() & setMask) ^ table.valueBits[value];
            port_type::toggle(v);
        }
        template<uint8_t V>
        static inline void set() {
            static_assert(V < table.valueBits.size, "pattern not possible");
            auto v = (port_type::get() & setMask) ^ table.valueBits[V];
            port_type::toggle(v);
        }
    private:
        PinSet() = delete;
    };
    
    template<AVR::Concepts::Port Port, uint8_t PinNumber>
    struct Pin final {
        static_assert(PinNumber < 8, "wrong pin number");
        Pin() = delete;
        typedef Port port;
        static constexpr uint8_t number = PinNumber;
        static constexpr std::byte pinMask{(1 << PinNumber)};
        static inline void on() __attribute__((always_inline)) {
            Port::get() |= pinMask; // single bit instruction sbi
        }
        static constexpr auto& high = on;
        static constexpr auto& pullup = on;
        static inline void off() __attribute__((always_inline)) { // single bit instruction cbi
            Port::get() &= ~pinMask;
        }
        static inline bool get() __attribute__((always_inline)) {
            return std::any(Port::get() & ~pinMask);
        }
        static constexpr auto& low = off;
        static inline void toggle() __attribute__((always_inline)){
            Port::template toggle<PinNumber>();
        }
        template<typename Dir>
        static inline void dir() {        
            Dir::template set<Port, pinMask>();
        }
        static inline bool read() __attribute__((always_inline)){
            return (Port::read() & pinMask) != std::byte{0};
        }
        static constexpr auto& isHigh = read;
    };
    
    /*
    template<AVR::Concepts::Pin Pin,  const std::chrono::microseconds& PulseWidth = Config::zeroMicroSeconds, bool ActiveLow = true>
    struct SinglePulse final {
        SinglePulse() {
            init();
        }    
        static void init() {
            if constexpr(ActiveLow) {
                Pin::high();
            }
            else {
                Pin::low();
            }
            Pin::template dir<Output>();
        }
        static void trigger(){
            if constexpr(ActiveLow) {
                Pin::low();
                if constexpr(PulseWidth != Config::zeroMicroSeconds) {
                    ::Util::delay(PulseWidth);                                
                }
                Pin::high();
            }
            else {
                Pin::high();
                if constexpr(PulseWidth != Config::zeroMicroSeconds) {
                    ::Util::delay(PulseWidth);                                
                }
                Pin::low();
            }
        }
    };
    */
    
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

    template<AVR::Concepts::Pin Pin, typename Dir>
    struct ActiveLow;
    
    template<AVR::Concepts::Pin Pin>
    struct ActiveLow<Pin, Output> {
        typedef Pin pin_type;
        typedef Output dir_type;
        static void init() {
            Pin::template dir<Output>();
        }
        static void activate() {
            Pin::low();
        }
        static void inactivate() {
            Pin::high();
        }
    };
    template<AVR::Concepts::Pin Pin>
    struct ActiveLow<Pin, Input> {
        typedef Pin pin_type;
        typedef Input dir_type;
        static void init() {
            Pin::template dir<Input>();
        }
        static bool isActive() {
            return !Pin::isHigh();
        }
    };

    template<AVR::Concepts::Pin Pin, typename Dir = AVR::Output>
    struct ActiveHigh;

    template<AVR::Concepts::Pin Pin>
    struct ActiveHigh<Pin, Output> {
        typedef Pin pin_type;
        typedef Output dir_type;
        static void init() {
            Pin::template dir<Output>();
        }
        static void activate() {
            Pin::high();
        }
        static void inactivate() {
            Pin::low();
        }    
        static bool activated() {
            return Pin::isHigh();
        }
    };
    template<AVR::Concepts::Pin Pin>
    struct ActiveHigh<Pin, Input> {
        typedef Pin pin_type;
        typedef Input dir_type;
        static void init() {
            Pin::template dir<Input>();
        }
        static bool isActive() {
            return Pin::isHigh();
        }
    };

    template<typename Pin>
    requires requires() {
        typename Pin::pin_type;
    }
    struct ScopedPin {
        ScopedPin() {
            Pin::activate();
        }
        ~ScopedPin() {
            Pin::inactivate();
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

