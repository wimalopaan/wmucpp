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

#include "../common/concepts.h"
#include "../common/isr.h"
#include "../common/delay.h"

#include "portmux.h"

namespace AVR {
    using namespace std::literals::chrono;

    template<typename PA>
    struct isNullPA : std::false_type {};
    
    template<typename T>
    struct isNullPA<External::Hal::NullProtocollAdapter<T>> : std::true_type {};
    
    struct FullDuplex final {};
    struct HalfDuplex final {};
    
    template<auto v>
    struct BaudRate final : etl::NamedConstant<v> {};
    
    template<auto v>
    struct ReceiveQueueLength final : etl::NamedConstant<v> {};
    
    template<auto v>
    struct SendQueueLength final : etl::NamedConstant<v> {};
    
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

    template<typename CN, typename  PA = External::Hal::NullProtocollAdapter<>, etl::Concepts::NamedFlag useISR = etl::NamedFlag<true>,
             etl::Concepts::NamedConstant RecvQLength = ReceiveQueueLength<64>, 
             etl::Concepts::NamedConstant SendQLength = SendQueueLength<64>, typename MCU = DefaultMcuType> class Usart;
    
    template<AVR::Concepts::ComponentNumber CN, typename PA, etl::Concepts::NamedFlag useISR,
             etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength , AVR::Concepts::AtMega MCU >
    class Usart<CN, PA, useISR, RecvQLength, SendQLength, MCU> final : public UsartBase<MCU, CN::value> {
        
        using Config = Project::Config;
        
        Usart() = delete;
    public:
        static inline constexpr auto N = CN::value;
        
        typedef Usart<CN, PA, useISR, RecvQLength, SendQLength> usart;
        typedef typename MCU::Usart      usart_type;
        typedef typename usart_type::SRA ucsra_type;
        typedef typename usart_type::SRB ucsrb_type;
        typedef typename usart_type::SRC ucsrc_type;
        typedef PA                       protocoll_adapter_type;

        typedef useISR use_isr_type;
        
        typedef typename std::conditional<useISR::value, volatile etl::FiFo<std::byte, SendQLength::value>, etl::FiFo<std::byte, SendQLength::value>>::type send_queue_type;
        typedef typename std::conditional<useISR::value, volatile etl::FiFo<std::byte, RecvQLength::value>, etl::FiFo<std::byte, RecvQLength::value>>::type recv_queue_type;

        static constexpr auto mcu_usart = getBaseAddr<typename MCU::Usart, N>;
        static_assert(N < MCU::Usart::count, "wrong number of usart");
        
      
        struct RxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::RXC> {
            friend usart;
            template<bool visible = useISR::value, typename = std::enable_if_t<visible>>
            inline static void
            isr() {
                isr_impl();
            }
        private:
            inline static void isr_impl() {
                const auto c = *mcu_usart()->udr;
                if constexpr (RecvQLength::value > 0) {
                    static_assert(std::is_same_v<PA, External::Hal::NullProtocollAdapter<>>, "recvQueue is used, no need for PA");
                    mRecvQueue.push_back(c);
                }
                else {
                    static_assert(RecvQLength::value == 0);
                    if constexpr(!std::is_same_v<PA, External::Hal::NullProtocollAdapter<>>) {
                        if (!PA::process(c)) {
                            assert("input not handled by protocoll adapter");
                        }
                    }
                    else {
                        static_assert(std::false_t<MCU>::value, "no recvQueue, no PA -> wrong configuration");
                    }
                }
            }
        };
        struct TxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::UDREmpty> {
            friend usart;
            template<bool visible = useISR::value, typename = std::enable_if_t<visible>>
            inline static void
            isr() {
                isr_impl();
            }
        private:
            static inline void isr_impl() {
                if (const auto c = mSendQueue.pop_front()) {
                    *mcu_usart()->udr = *c;;
                }
                else {
                    if constexpr(useISR::value) {
                        mcu_usart()->ucsrb.template clear<ucsrb_type::udrie, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                    }
                }
            }
        };
        
        template<bool visible = useISR::value, typename = std::enable_if_t<!visible>>
        inline static void
        periodic() {
            if (mcu_usart()->ucsra.template isSet<ucsra_type::rxc>()) {
                RxHandler::isr_impl();       
            }
            if (mcu_usart()->ucsra.template isSet<ucsra_type::udre>()) {
                TxHandler::isr_impl();
            }
        }
        
        template<etl::Concepts::NamedConstant Baud>
        inline static void init() {
            using namespace etl;
            static_assert(Baud::value >= 2400, "USART should use a valid baud rate >= 2400");
            
            if constexpr (Baud::value > 100000) {
                constexpr auto ubrr = ubrrValue2(Config::fMcu.value, Baud::value); 
//                using u = std::integral_constant<uint16_t, ubrr>;
//                u::_;
                mcu_usart()->ucsra.template add<ucsra_type::u2x>();
                if constexpr(AVR::Groups::isAtMega_8<MCU>::value) {
                    *mcu_usart()->ubbrh = 0x7f & (ubrr >> 8);
                    *mcu_usart()->ubbrl = ubrr;
                }
                else {
                    *mcu_usart()->ubbr = ubrr;
                }
            }
            else {
                constexpr auto ubrr = ubrrValue(Config::fMcu.value, Baud::value); 
//                using u = std::integral_constant<uint16_t, ubrr>;
//                u::_;
                if constexpr(AVR::Groups::isAtMega_8<MCU>::value) {
                    *mcu_usart()->ubbrh = 0x7f & (ubrr >> 8);
                    *mcu_usart()->ubbrl = ubrr;
                }
                else {
                    *mcu_usart()->ubbr = ubrr;
                }
            }            
            if constexpr(AVR::Groups::isAtMega_8<MCU>::value) {
                mcu_usart()->ucsrc.template add<ucsrc_type::ursel | ucsrc_type::ucsz1 | ucsrc_type::ucsz0, DisbaleInterrupt<NoDisableEnable>>();
            }
            else{
                mcu_usart()->ucsrc.template add<ucsrc_type::ucsz1 | ucsrc_type::ucsz0, DisbaleInterrupt<NoDisableEnable>>();
            }
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
                ::AVR::Util::delay(1_us);
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
        
        inline static constexpr uint16_t ubrrValue(uint32_t fcpu, uint32_t baud) {
            return (((1.0 * fcpu) / (16 * baud)) + 0.5) - 1;
        }
        inline static constexpr uint16_t ubrrValue2(uint32_t fcpu, uint32_t baud) {
            return (((1.0 * fcpu) / (8 * baud)) + 0.5) - 1;
        }
    };

    template<AVR::Concepts::ComponentPosition CP, AVR::Concepts::ProtocolAdapter PA, etl::Concepts::NamedFlag useISR,
             etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength , AVR::Concepts::At012DxSeries MCU >
    class Usart<CP, PA, useISR, RecvQLength, SendQLength, MCU> final : public UsartBase<MCU, CP::component_type::value> {
        
        static inline constexpr auto N = CP::component_type::value;
    public:
        static inline constexpr bool useInterrupts = useISR::value;
        static inline constexpr auto sendQLength = SendQLength::value;
        
        using component_type = CP::component_type;
        
    private:
        typedef typename std::conditional<useISR::value, volatile etl::FiFo<std::byte, SendQLength::value>, etl::FiFo<std::byte, SendQLength::value>>::type send_queue_type;
        typedef typename std::conditional<useISR::value, volatile etl::FiFo<std::byte, RecvQLength::value>, etl::FiFo<std::byte, RecvQLength::value>>::type recv_queue_type;

        static constexpr auto mcu_usart = getBaseAddr<typename MCU::Usart, N>;
        static_assert(N < AVR::Component::Count<typename MCU::Usart>::value, "wrong number of usart");

        using txpin = AVR::Portmux::Map<CP, MCU>::txpin;
        using rxpin = AVR::Portmux::Map<CP, MCU>::rxpin;
        
//        txpin::_;
//        rxpin::_;
        
        using ctrla_t = MCU::Usart::CtrlA_t;
        using ctrlb_t = MCU::Usart::CtrlB_t;
        using ctrlc_t = MCU::Usart::CtrlC_t;
        using status_t = MCU::Usart::Status_t;
        
        using usart = Usart<CP, PA, useISR, RecvQLength, SendQLength>;
        
        using Config = Project::Config;
        
        struct RxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::RXC> {
            friend usart;
            template<bool visible = useISR::value, typename = std::enable_if_t<visible>>
            inline static void
            isr() {
                isr_impl();
            }
        private:
            inline static void isr_impl() {
                const auto c = *mcu_usart()->rxd;
                if constexpr (RecvQLength::value > 0) {
                    static_assert(std::is_same_v<PA, External::Hal::NullProtocollAdapter<>>, "recvQueue is used, no need for PA");
                    mRecvQueue.push_back(c);
                }
                else {
                    static_assert(RecvQLength::value == 0);
//                    if constexpr(!isNullPA<PA>::value) {
                    if constexpr(!etl::is_same_v<PA, External::Hal::NullProtocollAdapter<>>) {
                        if (!PA::process(c)) {
                            assert("input not handled by protocoll adapter");
                        }
                    }
                    else {
                        static_assert(std::false_t<MCU>::value, "no recvQueue, no PA -> wrong configuration");
                    }
                }
            }
        };
        struct TxHandler : public IsrBaseHandler<typename AVR::ISR::Usart<N>::DRE> {
            friend usart;
            template<bool visible = useISR::value, typename = std::enable_if_t<visible>>
            inline static void
            isr() {
                isr_impl();
            }
        private:
            static inline void isr_impl() {
                if (const auto c = mSendQueue.pop_front()) {
                    mcu_usart()->status.template reset<status_t::txc>();
                    *mcu_usart()->txd = *c;
                }
                else {
                    if constexpr(useISR::value) {
                        mcu_usart()->status.template clear<status_t::dreif, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                    }
                }
            }
        };
    public:
        using protocoll_adapter_type = PA;

        inline static void lbmeDisable() {
            mcu_usart()->ctrla.template clear<ctrla_t::lbme, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
        }
        inline static void lbmeEnable() {
            mcu_usart()->ctrla.template add<ctrla_t::lbme, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
        }
        inline static void txOpenDrain() {
            mcu_usart()->ctrlb.template add<ctrlb_t::odme, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
        }
        inline static void txPinDisable() {
            txpin::template dir<AVR::Input>();
        }
        inline static void txPinEnable() {
            txpin::template dir<AVR::Output>();
        }
        template<bool on>
        inline static void txPinPullup() {
            txpin::template pullup<on>(); 
        }
        inline static void enableTransmit() {
            rxEnable<false>();
            txPinEnable();
        }
        inline static void enableReceive() {
            txPinDisable();
            rxEnable<true>();
        }
        template<etl::Concepts::NamedConstant Baud>
        inline static void initHDInv() {
            using namespace etl;
            static_assert(Baud::value >= 2400, "USART should use a valid baud rate >= 2400");

            txpin::on();
            mcu_usart()->ctrla.template add<ctrla_t::lbme, etl::DisbaleInterrupt<etl::NoDisableEnable>>();

            if constexpr (Baud::value > 100000) {
                constexpr auto ubrr = ubrrValue2(Config::fMcu.value, Baud::value);
                mcu_usart()->ctrlb.template add<ctrlb_t::rxm0, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                *mcu_usart()->baud = ubrr;
            }
            else {
                constexpr auto ubrr = ubrrValue(Config::fMcu.value, Baud::value);
                mcu_usart()->ctrlb.template clear<ctrlb_t::rxm0, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                *mcu_usart()->baud = ubrr;
            }

            mcu_usart()->ctrlc.template set<ctrlc_t::xfer8bit | ctrlc_t::noparity | ctrlc_t::xfer1stopbit>();
            mcu_usart()->ctrlb.template add<ctrlb_t::txen | ctrlb_t::rxen, etl::DisbaleInterrupt<etl::NoDisableEnable>>();

            // if constexpr(std::is_same_v<Mode, HalfDuplex>) {
            //     mcu_usart()->ctrlb.template clear<ctrlb_t::txen, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            //     mcu_usart()->ctrlb.template add<ctrlb_t::txen | ctrlb_t::rxen, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            // }
            txInvert(true);
        }

        template<etl::Concepts::NamedConstant Baud, typename Mode = FullDuplex, bool pullUp = true, auto flags = 0>
        inline static void init() {
            using namespace etl;
            static_assert(Baud::value >= 2400, "USART should use a valid baud rate >= 2400");
            
            if constexpr(std::is_same_v<Mode, FullDuplex>) {
                txpin::template dir<Output>();
                if constexpr(pullUp) {
                    rxpin::template pullup<true>(); 
                }
            }
            else if constexpr(std::is_same_v<Mode, HalfDuplex>) { 
                if constexpr(pullUp) {
                    txpin::template pullup<true>(); 
                }
                txpin::on();
               // txpin::template dir<Output>();
                mcu_usart()->ctrlb.template add<ctrlb_t::odme, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                mcu_usart()->ctrla.template add<ctrla_t::lbme, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            }
            else {
                static_assert(std::false_v<Mode>);
            }

            if constexpr (Baud::value > 100000) {
                constexpr auto ubrr = ubrrValue2(Config::fMcu.value, Baud::value); 
//                std::integral_constant<uint16_t, ubrr>::_;
                mcu_usart()->ctrlb.template add<ctrlb_t::rxm0, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                *mcu_usart()->baud = ubrr;
            }
            else {
                constexpr auto ubrr = ubrrValue(Config::fMcu.value, Baud::value); 
//                std::integral_constant<uint16_t, ubrr>::_;
//                std::integral_constant<uint32_t, Config::fMcu.value>::_;
//                std::integral_constant<uint16_t, Baud::value>::_;
                mcu_usart()->ctrlb.template clear<ctrlb_t::rxm0, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                *mcu_usart()->baud = ubrr;
            }            

            if constexpr(flags == 1) {
                mcu_usart()->ctrlc.template set<ctrlc_t::xfer8bit | ctrlc_t::evenparity | ctrlc_t::xfer2stopbit>();
            }
            else {
                mcu_usart()->ctrlc.template set<ctrlc_t::xfer8bit | ctrlc_t::noparity | ctrlc_t::xfer1stopbit>();
            }
            mcu_usart()->ctrlb.template add<ctrlb_t::txen | ctrlb_t::rxen, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
  
            if constexpr(std::is_same_v<Mode, HalfDuplex>) { 
                mcu_usart()->ctrlb.template clear<ctrlb_t::txen, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                mcu_usart()->ctrlb.template add<ctrlb_t::txen | ctrlb_t::rxen, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            }
        }
        
        template<bool visible = useISR::value, typename = std::enable_if_t<!visible>>
        inline static void
        periodic() {
            if (mcu_usart()->status.template isSet<status_t::rxc>()) {
                RxHandler::isr_impl();       
            }
            if (mcu_usart()->status.template isSet<status_t::dre>()) {
                TxHandler::isr_impl();
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
//                    mcu_usart()->ucsrb.template add<ucsrb_type::udrie>();
                }
                return true;
            }
            return false;
        }
        inline static void waitSendComplete() {
            while(!mSendQueue.empty()) {
                ::AVR::Util::delay(1_us);
            }
        }
        inline static bool isEmpty() {
            return mSendQueue.empty();
        }
        inline static bool isIdle() {
            return mcu_usart()->status.template isSet<status_t::txc>();
        }
        template<bool enable>
        inline static void rxEnable() {
            if constexpr (enable) {
                if(RecvQLength::value > 0) {
                    mRecvQueue.clear();
                }
                mcu_usart()->ctrlb.template add<ctrlb_t::rxen>();
            }
            else {
                mcu_usart()->ctrlb.template clear<ctrlb_t::rxen>();
            }
        }
        template<bool enable>
        inline static void txEnable() {
            if constexpr (enable) {
                if(SendQLength::value > 0) {
//                    decltype(mSendQueue)::_;
                    mSendQueue.clear();
                }
                mcu_usart()->ctrlb.template add<ctrlb_t::txen>();
            }
            else {
                mcu_usart()->ctrlb.template clear<ctrlb_t::txen>();
            }
        }
        inline static void rxInvert(const bool f) {
            if (f) {
                rxpin::template attributes<Meta::List<Attributes::Inverting<>>>();
            }
            else {
                rxpin::template attributes<Meta::List<Attributes::Reset<>>>();
            }
        }
        inline static void txInvert(const bool f) {
            if (f) {
                txpin::template attributes<Meta::List<Attributes::Inverting<>>>();
            }
            else {
                txpin::template attributes<Meta::List<Attributes::Reset<>>>();
            }
        }
    private:
        inline static send_queue_type mSendQueue;
        inline static recv_queue_type mRecvQueue;

        inline static constexpr uint16_t ubrrValue(uint32_t fcpu, uint32_t baud) {
            return (((64.0 * fcpu) / (16 * baud)) + 0.5) - 1;
        }
        inline static constexpr uint16_t ubrrValue2(uint32_t fcpu, uint32_t baud) {
            return (((64.0 * fcpu) / (8 * baud)) + 0.5) - 1;
        }
        
    };

}
