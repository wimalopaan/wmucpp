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
template<>
struct Ubrr<8000000, 19200> {
    static constexpr uint16_t value = 25;
};

template<int U>
struct UsartEventType;

template<>
struct UsartEventType<0> {
    UsartEventType() = delete;
    static constexpr EventType event = EventType::UsartRecv0;
    static constexpr EventType eventFe = EventType::UsartFe;
    static constexpr EventType eventUpe = EventType::UsartUpe;
    static constexpr EventType eventDor = EventType::UsartDor;
};
template<>
struct UsartEventType<1> {
    UsartEventType() = delete;
    static constexpr EventType event = EventType::UsartRecv1;
    static constexpr EventType eventFe = EventType::UsartFe;
    static constexpr EventType eventUpe = EventType::UsartUpe;
    static constexpr EventType eventDor = EventType::UsartDor;
};
template<>
struct UsartEventType<2> {
    UsartEventType() = delete;
    static constexpr EventType event = EventType::UsartRecv2;
    static constexpr EventType eventFe = EventType::UsartFe;
    static constexpr EventType eventUpe = EventType::UsartUpe;
    static constexpr EventType eventDor = EventType::UsartDor;
};

template<typename MCU, uint8_t N>
struct UsartBase {
    typedef MCU mcu_type;
    static constexpr uint8_t number = N;
};

template<uint8_t N, typename PA = void, typename MCU = DefaultMcuType>
class Usart final : public UsartBase<MCU, N>
{
    Usart() = delete;
public:
    typedef typename MCU::Usart usart_type;
    typedef typename usart_type::UCSRA ucsra_type;
    typedef typename usart_type::UCSRB ucsrb_type;
    typedef typename usart_type::UCSRC ucsrc_type;
    typedef PA protocoll_adapter_type;
    
    static constexpr auto mcu_usart = getBaseAddr<typename MCU::Usart, N>;
    static_assert(N < MCU::Usart::count, "wrong number of usart");

    struct RxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::RX> {
        static void isr() {
            const auto status = mcu_usart()->ucsra.template get<ucsra_type::fe | ucsra_type::upe | ucsra_type::dor>();
            const uint8_t c = *mcu_usart()->udr;
            if (isset(status)) {
                if (isset(status | ucsra_type::fe)) {
                    EventManager::enqueueISR({UsartEventType<N>::eventFe, N});
                }
                if (isset(status | ucsra_type::upe)) {
                    EventManager::enqueueISR({UsartEventType<N>::eventUpe, N});
                }
                if (isset(status | ucsra_type::dor)) {
                    EventManager::enqueueISR({UsartEventType<N>::eventDor, N});
                }
            } else {
                if constexpr (Config::Usart::RecvQueueLength > 0) {
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
                *mcu_usart()->udr = *c;;
            }
            else {
                mcu_usart()->ucsrb.template clear<ucsrb_type::udrie>();
            }
        }
    };
    template<uint32_t Baud>
    static void init() {
        static_assert(Baud >= 2400, "USART should use a valid baud rate >= 2400");
        *mcu_usart()->ubbr = Ubrr<Config::fMcu.value, Baud>::value;
        mcu_usart()->ucsrc.template add<ucsrc_type::ucsz1 | ucsrc_type::ucsz0>();
        mcu_usart()->ucsrb.template add<ucsrb_type::txen | ucsrb_type::rxen | ucsrb_type::rxcie>();
    }
    static bool get(uint8_t& item) {
        if constexpr (Config::Usart::RecvQueueLength > 0) {
            return recvQueue().pop_front(item);
        }
        else {
            return false;
        }
    }
    static std::optional<uint8_t> get() {
        if constexpr (Config::Usart::RecvQueueLength > 0) {
            return recvQueue().pop_front();
        }
        else {
            return {};
        }
    }
    static bool put(uint8_t item) {
        if(sendQueue().push_back(item)) {
            mcu_usart()->ucsrb.template add<ucsrb_type::udrie>();
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
        if constexpr (enable) {
            if(Config::Usart::RecvQueueLength > 0) {
                recvQueue().clear();
            }
            mcu_usart()->ucsrb.template add<ucsrb_type::rxen>();
        }
        else {
            mcu_usart()->ucsrb.template clear<ucsrb_type::rxen>();
        }
    }

private:
    static volatile std::FiFo<uint8_t, Config::Usart::SendQueueLength>& sendQueue() {
        static volatile std::FiFo<uint8_t, Config::Usart::SendQueueLength> mSendQueue;
        return mSendQueue;
    }
    static volatile std::FiFo<uint8_t, Config::Usart::RecvQueueLength>& recvQueue() {
        static volatile std::FiFo<uint8_t, Config::Usart::RecvQueueLength> mRecvQueue;
        return mRecvQueue;
    }
};

}
