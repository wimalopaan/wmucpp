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

#include <stdint.h>

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "container/fifo.h"
#include "hal/event.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"

template<uint8_t N, typename MCU = DefaultMcuType>
struct SWUsartRxTx;

// todo: den tx-Pin frei wählbar machen

template<>
struct SWUsartRxTx<0, AVR::ATMega1284P> {
    SWUsartRxTx() = delete;
    using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
    using rx = AVR::Pin<PortD, 6>; // icp1
    using tx = AVR::Pin<PortD, 7>;
};
template<>
struct SWUsartRxTx<1, AVR::ATMega1284P> {
    SWUsartRxTx() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using rx = AVR::Pin<PortB, 5>; // icp3      
    using tx = AVR::Pin<PortB, 1>;
};

template<>
struct SWUsartRxTx<0, AVR::ATMega328P> {
    SWUsartRxTx() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using rx = AVR::Pin<PortB, 0>; // icp1
    using tx = AVR::Pin<PortB, 1>;
};

template<>
struct SWUsartRxTx<1, AVR::ATMega328PB> {
    SWUsartRxTx() = delete;
    using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;
    using rx = AVR::Pin<PortE, 2>; // icp3
    using tx = AVR::Pin<PortE, 1>;
};

template<uint8_t N> struct SWUsartEventMapper;
template<>
struct SWUsartEventMapper<0> {
    SWUsartEventMapper() = delete;
    constexpr static EventType event = EventType::SwUsartRecv0;
};
template<>
struct SWUsartEventMapper<1> {
    SWUsartEventMapper() = delete;
    constexpr static EventType event = EventType::SwUsartRecv1;
};

template<uint8_t N, typename MCU = DefaultMcuType>
class SWUsart final {
    static_assert(N < 2, "wrong swusart number");

    static constexpr uint8_t mcu_timer_number = 2 * N + 1;

    static constexpr auto timer = AVR::getBaseAddr<typename MCU::Timer16Bit, mcu_timer_number>;
    static constexpr auto mcuInterrupts = AVR::getBaseAddr<typename MCU::Timer16Interrupts, mcu_timer_number>;

    typedef MCU mcu_type;
    typedef typename MCU::Timer16Bit mcu_timer_type;
    typedef typename MCU::Timer16Interrupts int_type;

    SWUsart() = delete;
public:
    struct TransmitBitHandler : public IsrBaseHandler<typename AVR::ISR::Timer<mcu_timer_number>::CompareA> {
        static void isr() {
            if (outframe != 0x0001) {
                if (outframe & 0x0001) {
                    SWUsartRxTx<N>::tx::on();
                }
                else {
                    SWUsartRxTx<N>::tx::off();
                }
                outframe = (outframe >> 1);
            }
            else {
                if (auto c = sendQueue.pop_front()) {
                    outframe = (3 << 9) | ((uint8_t)*c << 1);
                }
                else {
                    outframe = 0x0001;
                    mcuInterrupts()->timsk.template clear<int_type::Mask::ociea>();
                }
            }
        }
    };
    struct StartBitHandler : public IsrBaseHandler<typename AVR::ISR::Timer<mcu_timer_number>::Capture> {
        static void isr() {
            uint16_t ocra = *timer()->ocra;
            *timer()->ocrb = (*timer()->icr + ocra / 2) % ocra;
    
            mcuInterrupts()->tifr.template add<int_type::Flags::ocfb>();
            mcuInterrupts()->timsk.template clear<int_type::Mask::icie>();
            mcuInterrupts()->timsk.template add<int_type::Mask::ocieb>();
            inframe = 0;
            inbits = 0;
        }
    };
    struct ReceiveBitHandler : public IsrBaseHandler<typename AVR::ISR::Timer<mcu_timer_number>::CompareB> {
        static void isr() {
            inframe >>= 1;
            if (SWUsartRxTx<N>::rx::read()) {
                inframe |= (1 << 8);
            }
            ++inbits;
            if (inbits == 9) {
                uint8_t c = 0xff & (inframe >> 1);
                EventManager::enqueueISR({SWUsartEventMapper<N>::event, c});
                mcuInterrupts()->timsk.template clear<int_type::Mask::ocieb>();
                mcuInterrupts()->timsk.template add<int_type::Mask::icie>();
                mcuInterrupts()->tifr.template add<int_type::Flags::icf>();
                inframe = 0;
                inbits = 0;
            }
        }
    };

    template<uint16_t Baud>
    static void init() {
        static_assert(Baud >= 2400, "SWUSART should use a valid baud rate >= 2400");
        
        constexpr auto tsd = AVR::Util::calculate<AVR::Timer16Bit<mcu_timer_number>>(std::hertz{Baud});
        static_assert(tsd, "can't calculate timer setup");
        
        AVR::Timer16Bit<mcu_timer_number>::template prescale<tsd.prescaler>();
        
        *timer()->ocra= tsd.ocr;

        timer()->tccrb.template add<mcu_timer_type::TCCRB::wgm2>();
        timer()->tccrb.template add<mcu_timer_type::TCCRB::icnc>();
        timer()->tccrb.template clear<mcu_timer_type::TCCRB::ices>();

        mcuInterrupts()->timsk.template add<int_type::Mask::icie>();
        mcuInterrupts()->tifr.template add<int_type::Flags::icf | int_type::Flags::ocfb | int_type::Flags::ocfa>();

        SWUsartRxTx<N>::tx::template dir<AVR::Output>();
        SWUsartRxTx<N>::tx::on();

        SWUsartRxTx<N>::rx::template dir<AVR::Input>();
        SWUsartRxTx<N>::rx::on();
    }
    static bool put(uint8_t item) {
        if (sendQueue.push_back(item)) {
            mcuInterrupts()->timsk.template add<int_type::Mask::ociea>();
            return true;
        }
        return false;
    }
    template<bool enable>
    static void rxEnable() {
        mcuInterrupts()->tifr  |= _BV(ICF1);
        if constexpr (enable) {
            mcuInterrupts()->timsk = (mcuInterrupts()->timsk & ~(_BV(OCIE1B))) | _BV(ICIE1);
        }
        else {
            mcuInterrupts()->timsk &=  ~(_BV(OCIE1A));
            mcuInterrupts()->timsk &=  ~(_BV(ICIE1));
        }
    }
private:
    static std::FiFo<uint8_t, Config::Usart::SendQueueLength> sendQueue;
    static volatile uint16_t outframe;
    static volatile uint16_t inframe;
    static volatile uint8_t inbits;
};

template<uint8_t Timer, typename MCU>
std::FiFo<uint8_t, Config::Usart::SendQueueLength> SWUsart<Timer, MCU>::sendQueue;
template<uint8_t Timer, typename MCU>
volatile uint16_t SWUsart<Timer, MCU>::outframe = 0x0001;
template<uint8_t Timer, typename MCU>
volatile uint16_t SWUsart<Timer, MCU>::inframe = 0;
template<uint8_t Timer, typename MCU>
volatile uint8_t SWUsart<Timer, MCU>::inbits = 0;
