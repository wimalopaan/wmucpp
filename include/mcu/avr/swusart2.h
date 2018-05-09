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

#include <cstdint>

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "container/fifo.h"
#include "hal/event.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"

namespace SwUsart {
    // uses extrnale Ints 0 - 2
    template<uint8_t N, typename MCU = DefaultMcuType>
    struct IntParam;
    
    template<typename MCU>
    struct IntParam<0, MCU> {
        IntParam() = delete;
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
        using rx = AVR::Pin<PortD, 2>; // Int0
        typedef AVR::ISR::Int<0> int_type;
        inline static constexpr auto edge = MCU::Interrupt::EIControl::isc01;
        inline static constexpr auto mask = MCU::Interrupt::EIMask::int0;
        inline static constexpr auto flag = MCU::Interrupt::EIFlags::int0;
    };
    template<typename MCU>
    struct IntParam<1, MCU> {
        IntParam() = delete;
        using PortD = AVR::Port<typename MCU::PortRegister, AVR::D>;
        using rx = AVR::Pin<PortD, 3>; // Int1      
        typedef AVR::ISR::Int<1> int_type;
        inline static constexpr auto edge = MCU::Interrupt::EIControl::isc11;
        inline static constexpr auto mask = MCU::Interrupt::EIMask::int1;
        inline static constexpr auto flag = MCU::Interrupt::EIFlags::int1;
    };
    template<typename MCU>
    struct IntParam<2, MCU> {
        IntParam() = delete;
        using PortB = AVR::Port<typename MCU::PortRegister, AVR::B>;
        using rx = AVR::Pin<PortB, 2>; // Int2
        typedef AVR::ISR::Int<2> int_type;
        inline static constexpr auto edge = MCU::Interrupt::EIControl::isc21;
        inline static constexpr auto mask = MCU::Interrupt::EIMask::int2;
        inline static constexpr auto flag = MCU::Interrupt::EIFlags::int2;
    };
    
    template<uint8_t N, typename TX, typename PA, typename Timer, uint16_t Baud, ::Util::NamedConstant SendQLength = NamedConstant<0>, typename MCU = DefaultMcuType>
    class UsartInt;
    
    template<uint8_t N, typename TX, typename PA, typename Timer, uint16_t Baud, ::Util::NamedConstant SendQLength, typename MCU>
    requires AVR::ATMega_X8<MCU>() || AVR::ATMega_X4<MCU>()
    class UsartInt<N, TX, PA, Timer, Baud, SendQLength, MCU> final {
        static_assert(N <=2, "wrong swusart number");
        
        static constexpr auto timer = Timer::mcuTimer;
        static constexpr auto mcuInterrupts = Timer::mcuInterrupts;

        typedef typename MCU::Interrupt ext_interrupts;
        static constexpr auto extInterrupts = AVR::getBaseAddr<ext_interrupts>;
        
        typedef MCU mcu_type;
        typedef IntParam<N, MCU> param_type;
        typedef typename Timer::mcu_timer_type mcu_timer_type;
        typedef typename Timer::int_type int_type;
        typedef typename Timer::value_type value_type;
        
        UsartInt() = delete;
        public:
        struct TxHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::CompareA> {
            static void isr() {
                if (outframe != 0x0001) {
                    if (outframe & 0x0001) {
                        TX::on();
                    }
                    else {
                        TX::off();
                    }
                    outframe = (outframe >> 1);
                }
                else {
                    if (auto c = sendQueue.pop_front()) {
                        outframe = (3 << 9) | ((uint8_t)*c << 1);
                    }
                    else {
                        outframe = 0x0001;
                        mcuInterrupts()->timsk.template clear<int_type::Mask::ociea, DisbaleInterrupt<NoDisableEnable>>();
                    }
                }
            }
        };
        struct StartBitHandler : public IsrBaseHandler<typename param_type::int_type> {
            static void isr() {
                uint8_t ocrb = (*timer()->tcnt - (tsd.ocr / 10) + tsd.ocr / 2);
                if (ocrb >= tsd.ocr) {
                    ocrb -= tsd.ocr;
                }
                *timer()->ocrb = ocrb;
                
                mcuInterrupts()->tifr.template reset<int_type::Flags::ocfb>();
                mcuInterrupts()->timsk.template add<int_type::Mask::ocieb, DisbaleInterrupt<NoDisableEnable>>();

                extInterrupts()->eimsk.template clear<param_type::mask, DisbaleInterrupt<NoDisableEnable>>(); // disbale
                extInterrupts()->eifr.template reset<param_type::flag>(); // clear

                inframe = 0;
                inbits = 0;
            }
        };
        struct RxHandler : public IsrBaseHandler<typename AVR::ISR::Timer<Timer::number>::CompareB> {
            static void isr() {
                inframe >>= 1;
                if (param_type::rx::read()) {
                    inframe |= (1 << 8);
                }
                ++inbits;
                if (inbits == 9) {
                    uint8_t c = 0xff & (inframe >> 1);
                    if constexpr(!std::is_same<PA, void>::value) {
                        PA::process(std::byte{c});
                    }
                    else {
//                        EventManager::enqueueISR({SWUsartEventMapper<N>::event, std::byte{c}});
                    }
                    mcuInterrupts()->timsk.template clear<int_type::Mask::ocieb, DisbaleInterrupt<NoDisableEnable>>();
                    extInterrupts()->eimsk.template add<param_type::mask, DisbaleInterrupt<NoDisableEnable>>(); // enable
                    extInterrupts()->eifr.template reset<param_type::flag>(); // clear

                    inframe = 0;
                    inbits = 0;
                }
            }
        };
        
        static void init() {
            static_assert(Baud >= 2400, "SWUSART should use a valid baud rate >= 2400");
            static_assert(tsd, "can't calculate timer setup");
            
            Timer::template prescale<tsd.prescaler>();
            *timer()->ocra= tsd.ocr;
            timer()->tccra.template add<mcu_timer_type::TCCRA::wgm1, DisbaleInterrupt<NoDisableEnable>>();
            mcuInterrupts()->tifr.template reset<int_type::Flags::ocfb | int_type::Flags::ocfa>();
            
            extInterrupts()->eicra.template add<param_type::edge, DisbaleInterrupt<NoDisableEnable>>(); // falling
            extInterrupts()->eimsk.template add<param_type::mask, DisbaleInterrupt<NoDisableEnable>>(); // enable
            extInterrupts()->eifr.template reset<param_type::flag>(); // clear
            
            TX::template dir<AVR::Output>();
            TX::on();
            
            param_type::rx::template dir<AVR::Input>();
            param_type::rx::pullup();
        }
        static bool put(std::byte item) {
            static_assert(SendQLength::value > 0);
            if (sendQueue.push_back(item)) {
                mcuInterrupts()->timsk.template add<int_type::Mask::ociea>();
                return true;
            }
            return false;
        }
        template<bool enable>
        static void rxEnable() {
            mcuInterrupts()->tifr.template add<int_type::Flags::icf>();
            if constexpr (enable) {
                mcuInterrupts()->timsk.template add<int_type::Mask::ocieb>();
                extInterrupts()->eimsk.template add<MCU::Interrupt::EIMask::int2, DisbaleInterrupt<NoDisableEnable>>(); // enable
            }
            else {
                mcuInterrupts()->timsk.template clear<int_type::Mask::ociea>();
                extInterrupts()->eimsk.template clear<MCU::Interrupt::EIMask::int2, DisbaleInterrupt<NoDisableEnable>>(); // enable
            }
        }
        private:
        inline static constexpr auto tsd = AVR::Util::calculate<Timer>(std::hertz{Baud});
        inline static std::FiFo<std::byte, SendQLength::value> sendQueue;
        inline static volatile uint16_t outframe = 0x0001;
        inline static volatile uint16_t inframe = 0;
        inline static volatile uint8_t inbits = 0;
    };
    
}
