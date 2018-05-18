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
#include "util/util.h"
#include "hal/protocolladapter.h"
#include "hal/event.h"

namespace AVR {

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
    
    template<auto v>
    struct ReceiveQueueLength : NamedConstant<v> {};
    template<auto v>
    struct SendQueueLength : NamedConstant<v> {};
    
    template<typename MCU, uint8_t N>
    struct UsartBase {
        typedef MCU mcu_type;
        static constexpr uint8_t number = N;
    };
    
    namespace Util {
        template<typename T>
        struct RxHandler {
            typedef typename T::RxHandler type;    
        };
        template<>
        struct RxHandler<void> {
            typedef void type;
        };
        template<typename T>
        struct TxHandler {
            typedef typename T::TxHandler type;    
        };
        template<>
        struct TxHandler<void> {
            typedef void type;
        };
        
    } //!Util
    
    template<uint8_t N, typename PA = void, ::Util::NamedFlag useISR, ::Util::NamedFlag useEvents, 
             ::Util::NamedConstant RecvQLength, ::Util::NamedConstant SendQLength, typename MCU>
    class Usart final : public UsartBase<MCU, N>
    {
        Usart() = delete;
    public:
        typedef typename MCU::Usart usart_type;
        typedef typename usart_type::UCSRA ucsra_type;
        typedef typename usart_type::UCSRB ucsrb_type;
        typedef typename usart_type::UCSRC ucsrc_type;
        typedef PA protocoll_adapter_type;
        
        typedef typename std::conditional<useISR::value, volatile std::FiFo<std::byte, SendQLength::value>, std::FiFo<std::byte, SendQLength::value>>::type send_queue_type;
        typedef typename std::conditional<useISR::value, volatile std::FiFo<std::byte, RecvQLength::value>, std::FiFo<std::byte, RecvQLength::value>>::type recv_queue_type;
        
        static constexpr auto mcu_usart = getBaseAddr<typename MCU::Usart, N>;
        static_assert(N < MCU::Usart::count, "wrong number of usart");
    
        struct RxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::RX> {
            template<bool visible = useISR::value, typename = std::enable_if_t<visible>>
            inline static void
            isr() {
                isr_impl();
            }
        private:
            inline static void isr_impl() {
                if constexpr(useEvents::value) {
                    const auto status = mcu_usart()->ucsra.template get<ucsra_type::fe | ucsra_type::upe | ucsra_type::dor>();
                    const auto c = *mcu_usart()->udr;
                    if (isset(status)) {
                        if (isset(status | ucsra_type::fe)) {
                            EventManager::enqueueISR({UsartEventType<N>::eventFe, std::byte{N}});
                        }
                        if (isset(status | ucsra_type::upe)) {
                            EventManager::enqueueISR({UsartEventType<N>::eventUpe, std::byte{N}});
                        }
                        if (isset(status | ucsra_type::dor)) {
                            EventManager::enqueueISR({UsartEventType<N>::eventDor, std::byte{N}});
                        }
                    }
                    else {
                        if constexpr (RecvQLength::value > 0) {
                            if (mRecvQueue.push_back(c)) {
                                EventManager::enqueueISR({UsartEventType<N>::event, std::byte{c}});
                            }
                        }
                        else {
                            if constexpr(std::is_same<PA, void>::value) {
                                EventManager::enqueueISR({UsartEventType<N>::event, std::byte{c}});
                            }
                            else {
                                if (!PA::process(c)) {
                                    EventManager::enqueueISR({UsartEventType<N>::event, std::byte{c}});
                                }
                            }
                        }
                    }
                }
                else { // !useEvents
                    const auto c = *mcu_usart()->udr;
                    if constexpr (RecvQLength::value > 0) {
                        static_assert(std::is_same<PA, void>::value || std::is_same<PA, NullProtocollAdapter>::value, "recvQueue is used, no need for PA");
                        mRecvQueue.push_back(c);
                    }
                    else {
                        static_assert(RecvQLength::value == 0);
                        if constexpr(!std::is_same<PA, void>::value) {
                            if (!PA::process(c)) {
                                assert("input not handled by protocoll adapter");
                            }
                        }
                        else {
                            static_assert(std::false_t<MCU>::value, "no events, no recvQueue, no PA -> wrong configuration");
                        }
                    }
                }
            }
        };
        struct TxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::UDREmpty> {
            template<bool visible = useISR::value, typename = std::enable_if_t<visible>>
            inline static void
            isr() {
                isr_impl();
            }
        private:
            static inline void isr_impl() {
                if (auto c = mSendQueue.pop_front()) {
                    *mcu_usart()->udr = *c;;
                }
                else {
                    if constexpr(useISR::value) {
                        mcu_usart()->ucsrb.template clear<ucsrb_type::udrie, DisbaleInterrupt<NoDisableEnable>>();
                    }
                }
            }
        };

        template<bool Q = useISR::value>
        inline static 
        typename std::enable_if<!Q, void>::type
        periodic() {
            if (isset(mcu_usart()->ucsra.template get<ucsra_type::rxc>())) {
                RxHandler::isr_impl();       
            }
            if (isset(mcu_usart()->ucsra.template get<ucsra_type::udre>())) {
                TxHandler::isr_impl();
            }
        }
        
        template<uint32_t Baud>
        inline static void init() {
            static_assert(Baud >= 2400, "USART should use a valid baud rate >= 2400");
            constexpr auto ubrr = ubrrValue(Config::fMcu.value, Baud); 
            *mcu_usart()->ubbr = ubrr;
            mcu_usart()->ucsrc.template add<ucsrc_type::ucsz1 | ucsrc_type::ucsz0, DisbaleInterrupt<NoDisableEnable>>();
            mcu_usart()->ucsrb.template add<ucsrb_type::txen | ucsrb_type::rxen, DisbaleInterrupt<NoDisableEnable>>();
            if constexpr(useISR::value) {
                mcu_usart()->ucsrb.template add<ucsrb_type::rxcie, DisbaleInterrupt<NoDisableEnable>>();
            }
        }
        inline static bool get(std::byte& item) {
            if constexpr(RecvQLength::value > 0) {
                return mRecvQueue.pop_front(item);
            }
            else {
                return false;
            }
        }
        inline static std::optional<std::byte> get() {
            if constexpr(RecvQLength::value > 0) {
                return mRecvQueue.pop_front();
            }
            else {
                return {};
            }
        }
        inline static bool put(std::byte item) {
            if(mSendQueue.push_back(item)) {
                if constexpr(useISR::value) {
                    mcu_usart()->ucsrb.template add<ucsrb_type::udrie>();
                }
                return true;
            }
            return false;
        }
        inline static void waitSendComplete() {
            while(!mSendQueue.empty()) {
                ::Util::delay(1_us);
            }
        }
        inline static bool isEmpty() {
            return mSendQueue.empty();
        }
        template<bool enable>
        inline static void rxEnable() {
            if constexpr (enable) {
                if(RecvQLength::value > 0) {
                    mRecvQueue.clear();
                }
                mcu_usart()->ucsrb.template add<ucsrb_type::rxen>();
            }
            else {
                mcu_usart()->ucsrb.template clear<ucsrb_type::rxen>();
            }
        }
    private:        
        inline static send_queue_type mSendQueue;
        inline static recv_queue_type mRecvQueue;

        static constexpr uint16_t ubrrValue(uint32_t fcpu, uint32_t baud) {
            return (((1.0 * fcpu) / (16 * baud)) + 0.5) - 1;
        }
    };
}
