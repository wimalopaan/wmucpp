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

#include <cstdint>

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "container/fifo.h"
#include "hal/event.h"
#include "mcu/ports.h"

namespace AVR {
    
    template<typename useInt = MCU::UseInterrupts<true>, typename MCU = DefaultMcuType>
    struct SpiMaster final {
        SpiMaster() = delete;
        inline static constexpr bool useInterrupts = useInt::value;
        static constexpr auto spcr = [] {
            if constexpr(useInt::value) {
                return MCU::Spi::CR::spie | MCU::Spi::CR::spe | MCU::Spi::CR::mstr | MCU::Spi::CR::spr1 | MCU::Spi::CR::spr0;
            } else {
                return MCU::Spi::CR::spe | MCU::Spi::CR::mstr | MCU::Spi::CR::spr1 | MCU::Spi::CR::spr0;
            }
        }();
        typedef AVR::Output mosi_dir;
        typedef AVR::Input  miso_dir;
        typedef AVR::Output sck_dir;
        typedef AVR::Output ss_dir;
    };
    
    template<typename useInt = MCU::UseInterrupts<true>, typename MCU = DefaultMcuType> 
    struct SpiSlave final {
        SpiSlave() = delete;
        inline static constexpr bool useInterrupts = useInt::value;
        static constexpr auto spcr = []{
            if constexpr(useInt::value) {
                return MCU::Spi::CR::spie | MCU::Spi::CR::spe;
            }
            else {
                return MCU::Spi::CR::spe;
            }
        }();
        typedef AVR::Input  mosi_dir;
        typedef AVR::Output miso_dir;
        typedef AVR::Input  sck_dir;
        typedef AVR::Input  ss_dir;
    };
    
    template<int N> struct SpiEvent;
    
    template<>
    struct SpiEvent<0>{
        SpiEvent() = delete;
        static constexpr EventType event = EventType::Spi0;
    };
    template<>
    struct SpiEvent<1>{
        SpiEvent() = delete;
        static constexpr EventType event = EventType::Spi1;
    };
    
    template<int N, typename MCU> struct SpiPort;
    
    template<>
    struct SpiPort<0, AVR::ATMega1284P> {
        SpiPort() = delete;
        using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
        using mosi = AVR::Pin<PortB, 5>;
        using miso = AVR::Pin<PortB, 6>;
        using sck  = AVR::Pin<PortB, 7>;
        using ss  = AVR::Pin<PortB, 4>;
    };
    
    template<>
    struct SpiPort<0, AVR::ATMega328P> {
        SpiPort() = delete;
        using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
        using mosi = AVR::Pin<PortB, 3>;
        using miso = AVR::Pin<PortB, 4>;
        using sck  = AVR::Pin<PortB, 5>;
        using ss  = AVR::Pin<PortB, 2>;
    };
    
    template<typename Derived, bool Enable>
    struct SpiBase;
    template<typename Derived>
    struct SpiBase<Derived, true> {
        inline static bool overrun = false;
    };
    template<typename Derived>
    struct SpiBase<Derived, false> {
        inline static constexpr uint8_t number_of_flags = 1;
    };
    
    template<uint8_t N, typename Mode, typename Flags = void, typename MCU = DefaultMcuType>
    // fixme: ad-hoc requirement geht nicht, warum?
//    requires requires {
//        typename Mode::mosi_dir;
//    }
    class Spi final : public SpiBase<Spi<N, MCU>, std::is_same<Flags, void>::value>, 
            public std::conditional<Mode::useInterrupts, IsrBaseHandler<typename AVR::ISR::Spi<N>::Stc>, NoIsrBaseHandler>::type {
        
        static_assert(N < MCU::Spi::count, "wrong spi number");
        
        Spi() = delete;
        typedef SpiBase<Spi<N, MCU>, std::is_same<Flags, void>::value> base_type;
        inline static constexpr bool useBase = std::is_same<Flags, void>::value;
        
    public:
        typedef Mode mode;
        typedef SpiPort<N, MCU> spiPort;
        typedef MCU mcu_type;
        typedef typename MCU::Spi::SR flags_type;

        inline static constexpr const uint8_t number = N;
        
        inline static void init() {
            spiPort::mosi::template dir<typename Mode::mosi_dir>();
            spiPort::miso::template dir<typename Mode::miso_dir>();
            spiPort::sck::template dir<typename Mode::sck_dir>();
            spiPort::ss::template dir<typename Mode::ss_dir>();
            spiPort::ss::on();
            getBaseAddr<typename MCU::Spi, N>()->spcr.template set<Mode::spcr>();
        }
        inline static bool isReady() {
            return getBaseAddr<typename MCU::Spi, N>()->spsr.template isSet<MCU::Spi::SR::spif>();
        }
        inline static std::byte get() {
            return *getBaseAddr<typename MCU::Spi, N>()->spdr;
        }
        template<::Util::Callable<std::byte> Callable> 
        inline static void whenReady(const Callable& f){
            if (isReady()) {
                f(*getBaseAddr<typename MCU::Spi, N>()->spdr);
            }
        }
        template<bool sync = true>
        inline static bool put(std::byte c) {
//            if (!getBaseAddr<typename MCU::Spi, N>()->spsr.template isSet<MCU::Spi::SR::spif>()) {
//                return false;
//            }
            spiPort::ss::off();
            *getBaseAddr<typename MCU::Spi, N>()->spdr = c;
            if constexpr(sync) {
                while(!isReady());
            }
            return true;
        }
        inline static void off() {
            spiPort::ss::on();
        } 
        
        template<bool Q = Mode::useInterrupts>
        static 
        typename std::enable_if<Q, bool>::type
        leak() {
            if constexpr(useBase) {
                bool oBefore = base_type::overrun;
                base_type::overrun = false;
                return oBefore;
            }
            else {
                bool oBefore = Flags::isSet();
                Flags::reset();
                return oBefore;
            }
        }
        // SFINAE disable
        template<bool Q = Mode::useInterrupts>
        static 
        typename std::enable_if<Q, void>::type
        isr() {
            std::byte c = *getBaseAddr<typename MCU::Spi, N>()->spdr;
            if constexpr(useBase) {
                base_type::overrun |= !EventManager::enqueueISR({SpiEvent<N>::event, c});
            }
            else {
                bool ok = EventManager::enqueueISR({SpiEvent<N>::event, c});
                if (!ok) {
                    Flags::set();
                }
            }
        }
    private:
    };
}
