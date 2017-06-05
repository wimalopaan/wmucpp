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
struct SWUsartRxTx<0, AVR::ATMega328PB> {
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

template<>
struct SWUsartRxTx<0, AVR::ATTiny84> {
    SWUsartRxTx() = delete;
    using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using rx = AVR::Pin<PortA, 7>; // icp
    using tx = AVR::Pin<PortB, 2>;
};

template<>
struct SWUsartRxTx<0, AVR::ATTiny85> {
    SWUsartRxTx() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using rx = AVR::Pin<PortB, 2>; // int 0
    using tx = AVR::Pin<PortB, 3>;
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
class SWUsart;

template<uint8_t N, typename MCU>
requires AVR::ATMega_X8<MCU>() || ((N == 0) && AVR::ATTiny_X4<MCU>())
class SWUsart<N, MCU> final {
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
                EventManager::enqueueISR({SWUsartEventMapper<N>::event, std::byte{c}});
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
    static bool put(std::byte item) {
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
            mcuInterrupts()->timsk.template clear<int_type::Mask::ocieb | int_type::Mask::icie>();
        }
        else {
            mcuInterrupts()->timsk.template clear<int_type::Mask::ociea | int_type::Mask::icie>();
        }
    }
private:
    inline static std::FiFo<std::byte, Config::Usart::SendQueueLength> sendQueue;
    inline static volatile uint16_t outframe = 0x0001;
    inline static volatile uint16_t inframe = 0;
    inline static volatile uint8_t inbits = 0;
};


// todo: TxPin konfigurierbar machen
template<uint8_t N, AVR::ATTiny_X5 MCU>
class SWUsart<N, MCU> final {
    static_assert(N < 2, "wrong swusart number");
    
    static constexpr uint8_t mcu_timer_number = N;
    
    using mcu_timer_type = typename std::conditional<(N == 0), typename MCU::Timer8Bit, typename MCU::Timer8BitHighSpeed>::type;
    
    static constexpr auto timer = AVR::getBaseAddr<mcu_timer_type, mcu_timer_number>;
    static constexpr auto mcuInterrupts = AVR::getBaseAddr<typename MCU::TimerInterrupts>;

    typedef typename MCU::TimerInterrupts int_type;
public:
    SWUsart() = delete;

    template<uint16_t Baud>
    static void init() {
        static_assert(Baud >= 2400, "SWUSART should use a valid baud rate >= 2400");

        constexpr auto tsd = AVR::Util::calculate<AVR::Timer8Bit<mcu_timer_number>>(std::hertz{Baud});
        static_assert(tsd, "can't calculate timer setup");
        
        AVR::Timer8Bit<mcu_timer_number>::template prescale<tsd.prescaler>();
        *timer()->ocra = tsd.ocr;

        timer()->tccra.template add<mcu_timer_type::TCCRA::wgm1>();

        if constexpr(N == 0) {
            mcuInterrupts()->tifr.template add<int_type::Flags::ocf0a>();
        }
        else {
            mcuInterrupts()->tifr.template add<int_type::Flags::ocf1a>();
        }
        SWUsartRxTx<N>::tx::template dir<AVR::Output>();
        SWUsartRxTx<N>::tx::on();
    }
    static bool put(std::byte item) {
        if (sendQueue.push_back(item)) {
            if constexpr(N == 0) {
                mcuInterrupts()->timsk.template add<int_type::Mask::ocie0a>();
            }
            else {
                mcuInterrupts()->timsk.template add<int_type::Mask::ocie1a>();
            }
            return true;
        }
        return false;
    }
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
                    if constexpr(N == 0) {
                        mcuInterrupts()->timsk.template clear<int_type::Mask::ocie0a>();
                    }
                    else {
                        mcuInterrupts()->timsk.template clear<int_type::Mask::ocie1a>();
                    }
                }
            }
        }
    };
private:    
    inline static std::FiFo<std::byte, Config::Usart::SendQueueLength> sendQueue;
    inline static volatile uint16_t outframe = 0x0001;
};
