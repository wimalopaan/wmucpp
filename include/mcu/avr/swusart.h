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

#pragma once

#include <stdint.h>

#include "config.h"
#include "mcu/mcu.h"
#include "mcu/avr/isr.h"
#include "container/fifo.h"
#include "hal/event.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"

template<uint8_t N, typename MCU = DefaultMcuType> struct SWUsartRxTx;

// todo: testen auf icp3 bei atmega1284p

template<>
struct SWUsartRxTx<0, AVR::ATMega1284P> {
    SWUsartRxTx() = delete;
    using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
    using rx = AVR::Pin<PortD, 6>;
    using tx = AVR::Pin<PortD, 5>;
};
template<>
struct SWUsartRxTx<1, AVR::ATMega1284P> {
    SWUsartRxTx() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using rx = AVR::Pin<PortB, 5>;
    using tx = AVR::Pin<PortB, 1>;
};

template<>
struct SWUsartRxTx<0, AVR::ATMega328P> {
    SWUsartRxTx() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using rx = AVR::Pin<PortB, 0>;
    using tx = AVR::Pin<PortB, 1>;
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

template<uint16_t Baud> struct SWUsartOCRA;

template<>
struct SWUsartOCRA<2400> {
    SWUsartOCRA() = delete;
    static constexpr uint16_t prescaler = 64;
    static constexpr uint16_t value =  (Config::fMcu / 2400_Hz ) / prescaler;
};

// todo: besser den HW-Timer als template-parameter angeben
template<uint8_t N>
class SWUsart final {
    friend void ::TIMER1_COMPA_vect();
    friend void ::TIMER1_COMPB_vect();
    friend void ::TIMER1_CAPT_vect();

    static_assert(N < 2, "wrong swusart number");

    static constexpr uint8_t timerNumber = 2 * N + 1;

    static constexpr auto timer = AVR::getBaseAddr<DefaultMcuType::Timer16Bit, timerNumber>();
    static constexpr auto mcuInterrupts = AVR::getBaseAddr<typename DefaultMcuType::TimerInterrupts, timerNumber>();

public:
    SWUsart() = delete;

    template<uint16_t Baud>
    static void init() {
        static_assert(Baud >= 2400, "SWUSART should use a valid baud rate >= 2400");
        AVR::Timer16Bit<timerNumber>::template prescale<SWUsartOCRA<Baud>::prescaler>();
        timer->ocral = SWUsartOCRA<Baud>::value;
        timer->ocrah = (SWUsartOCRA<Baud>::value >> 8) & 0xff;
        timer->tccrb |= _BV(WGM12);
        timer->tccrb |= _BV(ICNC1);
        timer->tccrb &= ~_BV(ICES1);

        mcuInterrupts->timsk |= _BV(ICIE1);
        mcuInterrupts->tifr  |= _BV(ICF1) | _BV(OCF1B);
        mcuInterrupts->tifr  |= _BV(OCF1A);

        SWUsartRxTx<N>::tx::template dir<AVR::Output>();
        SWUsartRxTx<N>::tx::on();

        SWUsartRxTx<N>::rx::template dir<AVR::Input>();
        SWUsartRxTx<N>::rx::on();
    }
    static bool put(const uint8_t& item) {
        if (sendQueue.push_back(item)) {
            mcuInterrupts->timsk |= _BV(OCIE1A);
            return true;
        }
        return false;
    }
    template<bool e>
    static void rxEnable() {
        mcuInterrupts->tifr  |= _BV(ICF1);
        if (e) {
            mcuInterrupts->timsk = (mcuInterrupts->timsk & ~(_BV(OCIE1B))) | _BV(ICIE1);
        }
        else {
            mcuInterrupts->timsk &=  ~(_BV(OCIE1A));
            mcuInterrupts->timsk &=  ~(_BV(ICIE1));
        }
    }
private:
    static void isr_compa() {
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
                mcuInterrupts->timsk &=  ~(_BV(OCIE1A));
            }
        }
    }
    static void isr_icp() {
        uint16_t ocra = timer->ocra;
        timer->ocrb = (timer->icr + ocra / 2) % ocra;

        mcuInterrupts->tifr |= _BV(OCF1B);
        mcuInterrupts->timsk = (mcuInterrupts->timsk & ~(_BV(ICIE1))) | _BV(OCIE1B);
        inframe = 0;
        inbits = 0;
    }
    static void isr_compb() {
        inframe >>= 1;
        if (SWUsartRxTx<N>::rx::read()) {
            inframe |= (1 << 8);
        }
        ++inbits;
        if (inbits == 9) {
            uint8_t c = 0xff & (inframe >> 1);
            EventManager::enqueueISR({SWUsartEventMapper<N>::event, c});
            mcuInterrupts->timsk = (mcuInterrupts->timsk & ~(_BV(OCIE1B))) | _BV(ICIE1);
            mcuInterrupts->tifr |= _BV(ICF1);
            inframe = 0;
            inbits = 0;
        }
    }

    static std::FiFo<uint8_t, Config::Usart::SendQueueLength> sendQueue;
    static volatile uint16_t outframe;
    static volatile uint16_t inframe;
    static volatile uint8_t inbits;
};

template<uint8_t Timer>
std::FiFo<uint8_t, Config::Usart::SendQueueLength> SWUsart<Timer>::sendQueue;
template<uint8_t Timer>
volatile uint16_t SWUsart<Timer>::outframe = 0x0001;
template<uint8_t Timer>
volatile uint16_t SWUsart<Timer>::inframe = 0;
template<uint8_t Timer>
volatile uint8_t SWUsart<Timer>::inbits = 0;
