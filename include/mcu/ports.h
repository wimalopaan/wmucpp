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

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/concepts.h"
#include "std/traits.h"
#include "util/algorithm.h"
#include "util/disable.h"

#include "util/meta.h"

#include "std/types.h"

namespace AVR {
    
    struct Output final {
        Output() = delete;
        template<typename Port, std::byte mask>
        static inline void set() {
            Port::dir() |= mask;
        }
    };
    struct Input final {
        Input() = delete;
        template<typename Port, std::byte mask>
        static inline void set() {
            Port::dir() &= ~mask;
        }
    };
    
    template<typename MCUPort, MCU::Letter Name>
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
        static inline void toggle() { // AVR specific toggle mechanism: write "1" to PortInputRegister
            *getBaseAddr<MCUPort, Name>()->in |= std::byte{(1 << Bit)};
        }
        static inline void toggle(std::byte v) {
            *getBaseAddr<MCUPort, Name>()->in = v;
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
        static constexpr std::byte pinMasks[] = {Pins::pinMask...};
        static constexpr std::byte setMask = (Pins::pinMask | ... | std::byte{0});
        
//        static_assert(::Util::numberOfOnes(setMask) == size, "must use different pins in set");
        using pinlist = Meta::List<Pins...>;
        static_assert(Meta::is_set<pinlist>::value, "must use different pins in set"); // all Pins are different
        
        template<typename T> using port_from = typename T::port;
        using portlist = Meta::transform<port_from, pinlist>;
        typedef typename Meta::nth_element<0, portlist> port_type;
        static_assert(Meta::all_same<Meta::nth_element<0, portlist>, portlist>::value, "must use same port");
//        static_assert((std::is_same<port_type, typename Pins::port>::value && ... && true), "must use same port");
        
        static constexpr auto calculatePatterns = [](){
            constexpr uint16_t numberOfPatterns = (1 << size);
            std::array<std::byte, numberOfPatterns> data;
            for(uint8_t value = 0; value < numberOfPatterns; ++value) {
                std::byte pattern{0};
                std::byte vv{value};
                for(uint8_t bit = 0; bit < size; ++bit) {
                    if (std::any(vv & std::byte{0x01})) {
                        pattern |= pinMasks[bit];
                    }
                    vv >>= 1;
                }
                data[value] = pattern;
            }
            return data;
        };
        
        static constexpr auto valueBits = calculatePatterns();
        
        static inline void allOn() { // race-free 
            auto v = (port_type::get() & setMask) ^ setMask;
            port_type::toggle(v);
        }
        static inline void allPullup() {
            Scoped<DisbaleInterrupt<RestoreState>> di;
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
            static_assert(Meta::containsAll<pinlist, PP...>::value);
            constexpr std::byte mask = (PP::pinMask | ... | std::byte{0});
//            constexpr std::byte invertedMask = ~setMask;
//            static_assert((std::none(mask & invertedMask)), "Pin not in PinSet");
            
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
            static_assert(Meta::containsAll<pinlist, PP...>::value);
            constexpr std::byte mask = (PP::pinMask | ... | std::byte{0});
//            constexpr std::byte invertedMask = ~setMask;
//            static_assert((std::none(mask & invertedMask)), "Pin not in PinSet");
            Scoped<DisbaleInterrupt<RestoreState>> di;
            port_type::get() |= mask;
        }
        template<typename... PP>
        static inline void off() {
            static_assert(Meta::containsAll<pinlist, PP...>::value);
            constexpr std::byte mask = (PP::pinMask | ... | std::byte{0});
//            constexpr std::byte invertedMask = ~setMask;
//            static_assert((std::none(mask & invertedMask)), "Pin not in PinSet");
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
            (Pins::template dir<Dir>(),...);
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
        typedef bitsN_t<size> bits_type;
        static inline void set(bits_type value) {
            auto v = (port_type::get() & setMask) ^ valueBits[static_cast<typename bits_type::value_type>(value)]; // <> race-free for non-overlapping PinSets
            port_type::toggle(v);
        }
        static inline void set(uintN_t<size> value) {
            static_assert(std::numeric_limits<uintN_t<size>>::max() < valueBits.size);
            auto v = (port_type::get() & setMask) ^ valueBits[value];
            port_type::toggle(v);
        }
        template<uint8_t V>
        static inline void set() {
            static_assert(V < valueBits.size);
            auto v = (port_type::get() & setMask) ^ valueBits[V];
            port_type::toggle(v);
        }
    private:
        PinSet() = delete;
    };
    
    template<MCU::Port Port, uint8_t PinNumber>
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
    
    template<MCU::Pin Pin,  const std::microseconds& PulseWidth = Config::zeroMicroSeconds, bool ActiveLow = true>
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
        template<MCU::Pin Pin>
        static void activate() {
            Pin::low();
        }
        template<MCU::Pin Pin>
        static void inactivate() {
            Pin::high();
        }
    };
    struct ActiveHigh {
        template<MCU::Pin Pin>
        static void activate() {
            Pin::high();
        }
        template<MCU::Pin Pin>
        static void inactivate() {
            Pin::low();
        }    
    };
    
    template<MCU::Pin Pin, typename Mode>
    struct ScopedPin {
        ScopedPin() {
            Mode::template activate<Pin>();
        }
        ~ScopedPin() {
            Mode::template inactivate<Pin>();
        }
    };
    
}

//template<MCU::isPinSet PinSet, typename Commands>
//requires requires {typename Commands::commands; typename Commands::options;}
//struct CommandPort {
//    template<typename C, typename... OO>
//    requires 
//    requires(C c) {c.value;} &&
//    (requires(OO o) {typename OO::command_type; typename OO::value_type;} && ... && true) &&
//    (std::is_same<C, typename OO::command_type>::value && ... && true) &&
//    Meta::contains<typename Commands::commands, C>::value 
//    inline static void put(OO... options) {
//        auto ov = (options.value | ... | C::value);
//        PinSet::set(ov);
//    }

//    template<typename Part, typename C, typename... OO>
//    requires 
//    requires(C c) {c.value;} &&
//    requires(Part p) {typename Part::source_type;} &&
//    (requires(OO o) {typename OO::command_type; typename OO::value_type;} && ... && true) &&
//    (std::is_same<C, typename OO::command_type>::value && ... && true) &&
//    Meta::contains<typename Commands::commands, C>::value 
//    inline static void put(OO... options) {
//        typename Part::source_type ov = (options.value | ... | C::value);
//        PinSet::set(Part::convert(ov));
//    }
    
//    template<typename C, typename... OO>
//    requires 
//    requires(C c) {c.value;} &&
//    (std::is_same<C, typename OO::command_type>::value && ... && true) &&
//    (requires(OO o) {typename OO::command_type;} && ... && true)
//    static void get(OO... options) {
//        PinSet::read();
//    }
//};



template<typename Pin>
struct Set {
    static void input() {
        Pin::template dir<AVR::Input>();
    }
    static void output() {
        Pin::template dir<AVR::Output>();
    }
};

