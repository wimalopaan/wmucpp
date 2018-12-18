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
#include <type_traits>

#include <etl/fifo.h>
#include <etl/concepts.h>

#include <external/hal/protocolladapter.h>

//#include "config.h"
//#include "mcu/avr8.h"
//#include "container/fifo.h"
//#include "util/util.h"
//#include "hal/protocolladapter.h"
//#include "hal/event.h"

#include "../common/isr.h"

namespace AVR {
    
    template<auto v>
    struct ReceiveQueueLength : etl::NamedConstant<v> {};
    template<auto v>
    struct SendQueueLength : etl::NamedConstant<v> {};
    
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
    
    template<uint8_t N, typename PA, etl::Concepts::NamedFlag useISR,
             etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength, typename MCU>
    class Usart final : public UsartBase<MCU, N> {
        
        using Config = Project::Config;
        
        Usart() = delete;
    public:
        typedef typename MCU::Usart usart_type;
        typedef typename usart_type::SRA ucsra_type;
        typedef typename usart_type::SRB ucsrb_type;
        typedef typename usart_type::SRC ucsrc_type;
        typedef PA protocoll_adapter_type;
        
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
                const auto c = *mcu_usart()->udr;
                if constexpr (RecvQLength::value > 0) {
                    static_assert(std::is_same<PA, void>::value || std::is_same<PA, External::Hal::NullProtocollAdapter>::value, "recvQueue is used, no need for PA");
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
                        mcu_usart()->ucsrb.template clear<ucsrb_type::udrie, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
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
            using namespace etl;
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
        inline static etl::FiFo<std::byte, SendQLength::value> mSendQueue;
        inline static etl::FiFo<std::byte, RecvQLength::value> mRecvQueue;
        
        static constexpr uint16_t ubrrValue(uint32_t fcpu, uint32_t baud) {
            return (((1.0 * fcpu) / (16 * baud)) + 0.5) - 1;
        }
    };
}
