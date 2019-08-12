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

#include <cstddef>
#include <chrono>

#include <etl/types.h>

#include "../pgm/pgmarray.h"

// todo: überarbeiten

namespace AVR {
    struct UsePgmTable{}; // todo: named flag
    
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

    template<AVR::Concepts::Letter Name, typename MCU = DefaultMcuType>
    struct Port;

    template<AVR::Concepts::Letter Name, AVR::Concepts::At01Series MCU>
    struct Port<Name, MCU> final {
        using mcu = MCU;
        using mcuport_type = typename MCU::PortRegister;
        using name_type = Name;
        Port() = delete;
        static inline std::byte read() {
            return *getBaseAddr<mcuport_type , Name>()->in;
        }
        static inline volatile std::byte& dir() {
            return *getBaseAddr<mcuport_type , Name>()->dir;
        }
        static inline volatile std::byte& dirset() {
            return *getBaseAddr<mcuport_type , Name>()->dirset;
        }
        static inline volatile std::byte& get() {
            return *getBaseAddr<mcuport_type , Name>()->out;
        }
        static inline volatile std::byte& outset() {
            return *getBaseAddr<mcuport_type , Name>()->outset;
        }
        static inline volatile std::byte& outclear() {
            return *getBaseAddr<mcuport_type , Name>()->outclr;
        }
        static inline volatile std::byte& outtoggle() {
            return *getBaseAddr<mcuport_type , Name>()->outtgl;
        }
        template<auto Pin, bool on>
        static inline void pullup() {
            if constexpr(on) {
                getBaseAddr<mcuport_type , Name>()->pinctrl[Pin].template add<mcuport_type::PinCtrl_t::pullup>();
            }
            else {
                getBaseAddr<mcuport_type , Name>()->pinctrl[Pin].template clear<mcuport_type::PinCtrl_t::pullup>();
            }
        }
        template<auto Pin, auto value>
        static inline  void pinctrl() {
            getBaseAddr<mcuport_type , Name>()->pinctrl[Pin].template set<value>();
        }
        static inline volatile std::byte& intflags() {
            return *getBaseAddr<mcuport_type , Name>()->intflags;
        }
    };
    
    template<AVR::Concepts::Letter Name, AVR::Concepts::AtMega MCU>
    struct Port<Name, MCU> final {
        using mcu = MCU;
        using mcuport_type = typename MCU::PortRegister;
        using name_type = Name;

        Port() = delete;
        static inline void set(std::byte v) {
            *getBaseAddr<mcuport_type , Name>()->out = v;
        }
        template<std::byte V>
        static inline void set() {
            *getBaseAddr<mcuport_type , Name>()->out = V;
        }
        static inline volatile std::byte& get() {
            return *getBaseAddr<mcuport_type , Name>()->out;
        }
        static inline void dir(std::byte v) {
            *getBaseAddr<mcuport_type , Name>()->ddr = v;
        }
        template<uint8_t V>
        static inline void dir() {
            *getBaseAddr<mcuport_type , Name>()->ddr = V;
        }
        static inline volatile std::byte& dir() {
            return *getBaseAddr<mcuport_type , Name>()->ddr;
        }
        static inline std::byte read() {
            return *getBaseAddr<mcuport_type , Name>()->in;
        }
        template<uint8_t Bit, bool visible = !AVR::Groups::isAtMega_8<MCU>::value, typename = std::enable_if_t<visible>>
//        template<uint8_t Bit>
        static inline void toggle() { 
            static_assert(AVR::isSBICBICapable<mcuport_type , Name>(), "Port not sbi/cbi capable");
            *getBaseAddr<mcuport_type , Name>()->in |= std::byte{(1 << Bit)}; // AVR specific toggle mechanism: write "1" to PortInputRegister
        }
        static inline void toggle(std::byte v) {
            *getBaseAddr<mcuport_type , Name>()->in = v;
        }
        static inline constexpr uintptr_t address() {
            return reinterpret_cast<uintptr_t>(&getBaseAddr<mcuport_type , Name>()->out);
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
            using t = typename AVR::Util::Pgm::Converter<generator_type>::pgm_type;
            static inline constexpr auto valueBits = t{};
        };
    }

    template<typename PinList, typename F, typename MCU = DefaultMcuType>
    class PinSet;
    
    template<typename... PPs, typename PgmFlag, AVR::Concepts::AtMega MCU>
    class PinSet<Meta::List<PPs...>, PgmFlag, MCU> final {
    public:
        static constexpr bool usePgm = std::is_same_v<PgmFlag, UsePgmTable>;
        
        using pinlist = Meta::List<PPs...>;
        
        using table_type = detail::PinSet::Table<usePgm, PPs...>;
        static inline constexpr table_type table{};
        
        using gen_type = typename table_type::generator_type;
        
        static_assert(Meta::size<pinlist>::value <= 8, "too much pins in PinSet");
        static constexpr uint8_t size = Meta::size<pinlist>::value;
        static constexpr auto pinMasks = gen_type::pinMasks;
        static constexpr std::byte setMask = []{
                return (PPs::pinMask | ... | std::byte{0});
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
            (PPs::template dir<Dir>(),...);
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
    
    template<AVR::Concepts::Pin... Pin>
    inline void on() {
        (Pin::on(), ...);
    }

    template<AVR::Concepts::Pin... Pin>
    inline void off() {
        (Pin::off(), ...);
    }

    template<typename PinList, typename MCU = DefaultMcuType>
    struct PinGroup;
    
    namespace detail {
        template<typename Pin>
        struct getPort {
            using type = typename Pin::port;
        };
    }
    
    template<typename... Pins, AVR::Concepts::At01Series MCU>
    requires Meta::all_same_front_v<Meta::transform_type<detail::getPort, Meta::List<Pins...>>>
    struct PinGroup<Meta::List<Pins...>, MCU> {
        using pin_list = Meta::List<Pins...>;
        
        inline static constexpr std::byte group_value = (Pins::pinMask | ...);
        
        using port = Meta::front<pin_list>::port;   

        inline static std::byte read() {
            return port::read() & group_value;
        }
        inline static void on() {
            port::outset() = group_value;
        }
        inline static void off() {
            port::outclear() = group_value;
        }
        inline static void pullup() {
            (Pins::template pullup<true>(), ...);
        }
        template<typename AList>
        static inline void attributes() {
            (Pins::template attributes<AList>(),...);
        }
        static inline void onInterrupt(auto f) {
            f();
            port::intflags() = group_value;
        }
        template<typename Dir>
        inline static void dir() {
            if constexpr(std::is_same_v<Dir, AVR::Output>) {
                port::dirset() = group_value;
            }
        }
    };
    
    namespace Attributes {
        template<typename MCU = DefaultMcuType>
        struct Inverting : std::integral_constant<typename MCU::PortRegister::PinCtrl_t, MCU::PortRegister::PinCtrl_t::inven> {}; 
        struct OnRising;
        struct OnFalling;
        struct BothEdges;
        template<typename Kind, typename MCU = DefaultMcuType>
        struct Interrupt;
        template<typename MCU>
        struct Interrupt<BothEdges, MCU> : std::integral_constant<typename MCU::PortRegister::PinCtrl_t, MCU::PortRegister::PinCtrl_t::bothedges> {}; 
        template<typename MCU>
        struct Interrupt<OnRising, MCU> : std::integral_constant<typename MCU::PortRegister::PinCtrl_t, MCU::PortRegister::PinCtrl_t::rising> {}; 
        template<typename MCU>
        struct Interrupt<OnFalling, MCU> : std::integral_constant<typename MCU::PortRegister::PinCtrl_t, MCU::PortRegister::PinCtrl_t::falling> {}; 
        template<typename MCU = DefaultMcuType>
        struct DigitalDisable : std::integral_constant<typename MCU::PortRegister::PinCtrl_t, MCU::PortRegister::PinCtrl_t::disable> {}; 
    }
    
    template<AVR::Concepts::Port Port, uint8_t PinNumber, typename MCU = DefaultMcuType>
    struct Pin;
    
    template<AVR::Concepts::Port Port, uint8_t PinNumber, AVR::Concepts::At01Series MCU>
    struct Pin<Port, PinNumber, MCU> final {
        static_assert(PinNumber < 8, "wrong pin number");
        Pin() = delete;
        using mcu = typename Port::mcu;
        typedef Port port;
        
        static inline constexpr uint8_t number = PinNumber;
        static inline constexpr std::byte pinMask{(1 << PinNumber)};
        static inline void on() {
            Port::outset() = pinMask; 
        }
        static inline constexpr auto& high = on;
        static inline void off() {
            Port::outclear() = pinMask; 
        }
        static inline constexpr auto& low = off;
        static inline void toggle() {
            Port::outtoggle() = pinMask; 
        }
        template<bool On>
        static inline void pullup() {
            Port::template pullup<PinNumber, On>();
        }
        static inline bool read() {
            return (Port::read() & pinMask) != std::byte{0};
        }
        static constexpr auto& isHigh = read;
        
        template<typename Dir>
        static inline void dir() {        
            if constexpr(std::is_same_v<Dir, AVR::Output>) {
                port::dirset() = pinMask;
            }
        }
        
        template<typename AList>
        static inline void attributes() {
            []<typename... AA>(Meta::List<AA...>) {
                constexpr auto v = (AA::value || ...);
                port::template pinctrl<PinNumber, v>();
            }(AList{});
        }
        static inline void resetInt() {
            Port::intflags() = pinMask; 
        }
        
        
    };

    template<AVR::Concepts::Port Port, uint8_t PinNumber, AVR::Concepts::AtMega MCU>
    struct Pin<Port, PinNumber, MCU> final {
        static_assert(PinNumber < 8, "wrong pin number");
        Pin() = delete;
        using mcu = typename Port::mcu;
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
            return std::any(Port::get() & pinMask);
        }
        static constexpr auto& low = off;

        template<bool visible = !AVR::Groups::isAtMega_8<mcu>::value, typename = std::enable_if_t<visible>>
        static inline void toggle() {
            Port::template toggle<PinNumber>();
        }
        
        template<bool visible = AVR::Groups::isAtMega_8<mcu>::value, typename = std::enable_if_t<visible>>
        static inline void toggleWithOnOff() {
            if (get()) {
                off();
            }
            else {
                on();
            }
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
        inline static void init() {
            inactivate();
            Pin::template dir<Output>();
        }
        inline static void activate() {
            Pin::low();
        }
        inline static void inactivate() {
            Pin::high();
        }
    };
    template<AVR::Concepts::Pin Pin>
    struct ActiveLow<Pin, Input> {
        typedef Pin pin_type;
        typedef Input dir_type;
        inline static void init() {
            Pin::template dir<Input>();
            Pin::pullup();
        }
        inline static bool isActive() {
            return !Pin::isHigh();
        }
    };

    template<AVR::Concepts::Pin Pin, typename Dir = AVR::Output>
    struct ActiveHigh;

    template<AVR::Concepts::Pin Pin>
    struct ActiveHigh<Pin, Output> {
        typedef Pin pin_type;
        typedef Output dir_type;
        inline static void init() {
            inactivate();
            Pin::template dir<Output>();
        }
        inline static void activate() {
            Pin::high();
        }
        inline static void inactivate() {
            Pin::low();
        }    
        inline static bool activated() {
            return Pin::isHigh();
        }
        inline static void toggle() {
            Pin::toggle();
        }
    };
    template<AVR::Concepts::Pin Pin>
    struct ActiveHigh<Pin, Input> {
        typedef Pin pin_type;
        typedef Input dir_type;
        inline static void init() {
            Pin::template dir<Input>();
        }
        inline static bool isActive() {
            return Pin::isHigh();
        }
    };

    template<typename Pin>
    requires requires() {
        typename Pin::pin_type;
    }
    struct ScopedPin {
        inline ScopedPin() {
            Pin::activate();
        }
        inline ~ScopedPin() {
            Pin::inactivate();
        }
    };
}

template<typename Pin>
struct Set {
    inline static void input() {
        Pin::template dir<AVR::Input>();
    }
    inline static void output() {
        Pin::template dir<AVR::Output>();
    }
};

