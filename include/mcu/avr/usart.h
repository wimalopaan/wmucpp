/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "util/util.h"
#include "hal/event.h"

namespace AVR {

template<uint32_t Fcpu, uint32_t Baud>
struct Ubrr;

template<>
struct Ubrr<20000000, 2400> {
    static constexpr uint16_t value = 520;
};
template<>
struct Ubrr<20000000, 9600> {
    static constexpr uint16_t value = 129;
};
template<>
struct Ubrr<20000000, 19200> {
    static constexpr uint16_t value = 64;
};
template<>
struct Ubrr<20000000, 115200> {
    static constexpr uint16_t value = 10;
};

template<>
struct Ubrr<16000000, 2400> {
    static constexpr uint16_t value = 416;
};
template<>
struct Ubrr<16000000, 9600> {
    static constexpr uint16_t value = 103;
};
template<>
struct Ubrr<16000000, 19200> {
    static constexpr uint16_t value = 51;
};

template<int U>
struct UsartEventType;

template<>
struct UsartEventType<0> {
    UsartEventType() = delete;
    static constexpr EventType event = EventType::UsartRecv0;
    static constexpr EventType eventFe = EventType::UsartFe0;
    static constexpr EventType eventUpe = EventType::UsartUpe0;
    static constexpr EventType eventDor = EventType::UsartDor0;
};
template<>
struct UsartEventType<1> {
    UsartEventType() = delete;
    static constexpr EventType event = EventType::UsartRecv1;
    static constexpr EventType eventFe = EventType::UsartFe1;
    static constexpr EventType eventUpe = EventType::UsartUpe1;
    static constexpr EventType eventDor = EventType::UsartDor1;
};
template<>
struct UsartEventType<2> {
    UsartEventType() = delete;
    static constexpr EventType event = EventType::UsartRecv2;
    static constexpr EventType eventFe = EventType::UsartFe2;
    static constexpr EventType eventUpe = EventType::UsartUpe2;
    static constexpr EventType eventDor = EventType::UsartDor2;
};

template<typename MCU, uint8_t N>
struct UsartBase {
    typedef MCU mcu_type;
    static constexpr uint8_t number = N;
    static constexpr auto mcu_usart = getBaseAddr<typename MCU::Usart, N>;
};

template<uint8_t N, typename PA = void, typename MCU = DefaultMcuType>
class Usart : public UsartBase<MCU, N>
{
    friend void ::USART_RX_vect();
    friend void ::USART_UDRE_vect();
    friend void ::USART0_RX_vect();
    friend void ::USART0_UDRE_vect();
    friend void ::USART1_RX_vect();
    friend void ::USART1_UDRE_vect();
    
    Usart() = delete;
    
public:
    static constexpr auto mcu_usart = UsartBase<MCU, N>::mcu_usart;
    typedef PA protocoll_adapter_type;
    static_assert(N < MCU::Usart::count, "wrong number of usart");

    struct RxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::RX> {
        static void isr() {
            if (mcu_usart()->ucsra & (_BV(FE0) | _BV(UPE0) | _BV(DOR0))) {
                if (mcu_usart()->ucsra & _BV(FE0)) {
                    mcu_usart()->udr;
                    EventManager::enqueueISR({UsartEventType<N>::eventFe, 0});
                }
                if (mcu_usart()->ucsra & _BV(UPE0)) {
                    mcu_usart()->udr;
                    EventManager::enqueueISR({UsartEventType<N>::eventUpe, 0});
                }
                if (mcu_usart()->ucsra & _BV(DOR0)) {
                    mcu_usart()->udr;
                    EventManager::enqueueISR({UsartEventType<N>::eventDor, 0});
                }
            } else {
                const uint8_t c = mcu_usart()->udr;
                if(Config::Usart::RecvQueueLength > 0) {
                    if (recvQueue().push_back(c)) {
                        EventManager::enqueueISR({UsartEventType<N>::event, c});
                    }
                }
                else {
                    if constexpr(std::is_same<PA, void>::value) {
                        EventManager::enqueueISR({UsartEventType<N>::event, c});
                    }
                    else {
                        if (!PA::process(c)) {
                            EventManager::enqueueISR({UsartEventType<N>::event, c});
                        }
                    }
                }
            }
        }
    };
    struct TxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::UDREmpty> {
        static void isr() {
            if (auto c = sendQueue().pop_front()) {
                mcu_usart()->udr = *c;;
            }
            else {
                mcu_usart()->ucsrb &= ~_BV(UDRIE0);
            }
        }
    };

    
    template<uint32_t Baud>
    static void init() {
        static_assert(Baud >= 2400, "USART should use a valid baud rate >= 2400");
        mcu_usart()->ubbr = Ubrr<Config::fMcu.value, Baud>::value;
        mcu_usart()->ucsrb |= _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0);
        getBaseAddr<typename MCU::Usart, N>()->ucsrc |= _BV(UCSZ01) | _BV(UCSZ00);
    }
    static bool get(uint8_t& item) {
        if (Config::Usart::RecvQueueLength > 0) {
            return recvQueue().pop_front(item);
        }
        else {
            return false;
        }
    }
    static std::optional<uint8_t> get() {
        if (Config::Usart::RecvQueueLength > 0) {
            return recvQueue().pop_front();
        }
        else {
            return {};
        }
    }
    static bool put(uint8_t item) {
        if(sendQueue().push_back(item)) {
            getBaseAddr<typename MCU::Usart, N>()->ucsrb |= _BV(UDRIE0);
            return true;
        }
        return false;
    }
    static void waitSendComplete() {
        while(!sendQueue().empty()) {
            ::Util::delay(1_us);
        }
    }
    static bool isEmpty() {
        return sendQueue().empty();
    }

    template<bool enable>
    static void rxEnable() {
        if (enable) {
            if(Config::Usart::RecvQueueLength > 0) {
                recvQueue().clear();
            }
            getBaseAddr<typename MCU::Usart, N>()->ucsrb |= _BV(RXEN0);
        }
        else {
            getBaseAddr<typename MCU::Usart, N>()->ucsrb &= ~_BV(RXEN0);
        }
    }

private:
    static std::FiFo<uint8_t, Config::Usart::SendQueueLength>& sendQueue() {
        static std::FiFo<uint8_t, Config::Usart::SendQueueLength> mSendQueue;
        return mSendQueue;
    }
    static std::FiFo<uint8_t, Config::Usart::RecvQueueLength>& recvQueue() {
        static std::FiFo<uint8_t, Config::Usart::RecvQueueLength> mRecvQueue;
        return mRecvQueue;
    }

//    inline static void rx_isr() {
//        if (getBaseAddr<typename MCU::Usart, N>()->ucsra & (_BV(FE0) | _BV(UPE0) | _BV(DOR0))) {
//            if (getBaseAddr<typename MCU::Usart, N>()->ucsra & _BV(FE0)) {
//                getBaseAddr<typename MCU::Usart, N>()->udr;
//                EventManager::enqueueISR({UsartEventType<N>::eventFe, 0});
//            }
//            if (getBaseAddr<typename MCU::Usart, N>()->ucsra & _BV(UPE0)) {
//                getBaseAddr<typename MCU::Usart, N>()->udr;
//                EventManager::enqueueISR({UsartEventType<N>::eventUpe, 0});
//            }
//            if (getBaseAddr<typename MCU::Usart, N>()->ucsra & _BV(DOR0)) {
//                getBaseAddr<typename MCU::Usart, N>()->udr;
//                EventManager::enqueueISR({UsartEventType<N>::eventDor, 0});
//            }
//        } else {
//            const uint8_t c = getBaseAddr<typename MCU::Usart, N>()->udr;
//            if(Config::Usart::RecvQueueLength > 0) {
//                if (recvQueue().push_back(c)) {
//                    EventManager::enqueueISR({UsartEventType<N>::event, c});
//                }
//            }
//            else {
//                if constexpr(std::is_same<PA, void>::value) {
//                    EventManager::enqueueISR({UsartEventType<N>::event, c});
//                }
//                else {
//                    if (!PA::process(c)) {
//                        EventManager::enqueueISR({UsartEventType<N>::event, c});
//                    }
//                }
//            }
//        }
//    }
//    inline static void tx_isr() {
//        if (auto c = sendQueue().pop_front()) {
//            getBaseAddr<typename MCU::Usart, N>()->udr = *c;;
//        }
//        else {
//            getBaseAddr<typename MCU::Usart, N>()->ucsrb &= ~_BV(UDRIE0);
//        }
//    }
};

}
