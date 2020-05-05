#pragma once

#include <cstdint>

#include <mcu/common/isr.h>
#include <mcu/common/concepts.h>

namespace External {
    namespace SoftSerial {

        template<typename Pins, typename Timer, AVR::Concepts::ProtocolAdapter PA = External::Hal::NullProtocollAdapter,
                 etl::Concepts::NamedConstant Baud = AVR::BaudRate<9600>,
                 etl::Concepts::NamedConstant RecvQLength = AVR::ReceiveQueueLength<64>, 
                 etl::Concepts::NamedConstant SendQLength = AVR::SendQueueLength<64>, 
                 etl::Concepts::NamedFlag Inverted = etl::NamedFlag<false>,
                 AVR::Concepts::Pin dbg = AVR::NoPin,
                 typename MCU = DefaultMcuType> 
        struct Usart;

        template<AVR::Concepts::Pin RxPin, AVR::Concepts::Pin TxPin, auto N, AVR::Concepts::ProtocolAdapter PA, 
                 etl::Concepts::NamedConstant Baud,
                 etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength, 
                 etl::Concepts::NamedFlag Inverted,
                 typename Dbg,
                 AVR::Concepts::At01Series MCU>
        struct Usart<Meta::List<RxPin, TxPin>, AVR::Component::Tcd<N>, PA, Baud, 
                RecvQLength, SendQLength, Inverted, Dbg, MCU> final {

            static inline constexpr bool useInterrupts = true;
            
            using mcu_timer_t = typename MCU::TCD; 
            static constexpr auto mcu_tcd = AVR::getBaseAddr<mcu_timer_t, N>;

            using CtrlA2_t = mcu_timer_t::CtrlA2_t;
            using CtrlA4_t = mcu_timer_t::CtrlA4_t;
            using CtrlB_t = mcu_timer_t::CtrlB_t;
            using CtrlC_t = mcu_timer_t::CtrlC_t;
            using CtrlE_t = mcu_timer_t::CtrlE_t;
            using Status_t = mcu_timer_t::Status_t;
   
            static inline auto status_r = []()->auto&{return mcu_tcd()->status;};
            static inline auto ctrla_r = []()->auto&{return mcu_tcd()->ctrla;};
            static inline auto ctrlb_r = []()->auto&{return mcu_tcd()->ctrlb;};
            static inline auto ctrlc_r = []()->auto&{return mcu_tcd()->ctrlc;};
            static inline auto ctrle_r = []()->auto&{return mcu_tcd()->ctrle;};
            
            using gpior_t = typename MCU::Gpior; 
            static inline volatile std::byte data;
            static inline volatile uint8_t bitCount;
            
            using rxPinAttrInt = std::conditional_t<Inverted::value, 
                                                 Meta::List<AVR::Attributes::Inverting<>, AVR::Attributes::Interrupt<AVR::Attributes::OnFalling>>,
                                                 Meta::List<AVR::Attributes::Pullup<MCU>, AVR::Attributes::Interrupt<AVR::Attributes::OnFalling>>>;
            using rxPinAttrNoInt = std::conditional_t<Inverted::value, 
                                                 Meta::List<AVR::Attributes::Inverting<>>,
                                                 Meta::List<AVR::Attributes::Pullup<MCU>>>;
            
            static inline constexpr auto sendQLength = SendQLength::value;
            
            static inline constexpr auto flags = AVR::getBaseAddr<gpior_t, 0>;
            static inline constexpr uint8_t startMask = 0x01;
            static inline constexpr uint8_t rxMask = 0x02;
            static inline constexpr uint8_t recvMask = 0x04;
            static inline constexpr uint8_t sendMask = 0x08;
            
            static inline bool isStartBit() {
                return (flags()->data & startMask);
            }
            static inline void clearStartBit() {
                flags()->data = flags()->data & ~startMask;
            }
            static inline void setStartBit() {
                flags()->data = flags()->data | startMask;
            }
            static inline bool isRxEn() {
                return (flags()->data & rxMask);
            }
            static inline void clearRxEn() {
                flags()->data = flags()->data & ~rxMask;
            }
            static inline void setRxEn() {
                flags()->data = flags()->data | rxMask;
            }
            static inline bool isReceiving() {
                return (flags()->data & recvMask);
            }
            static inline void clearReceiving() {
                flags()->data = flags()->data & ~recvMask;
            }
            static inline void setReceiving() {
                flags()->data = flags()->data | recvMask;
            }
            static inline bool isSending() {
                return (flags()->data & sendMask);
            }
            static inline void clearSending() {
                flags()->data = flags()->data & ~sendMask;
            }
            static inline void setSending() {
                flags()->data = flags()->data | sendMask;
            }
            
            static inline void clearBits() {
                flags()->data = 0x00;
            }
            
            static inline constexpr uint16_t period = Project::Config::fMcu.value / Baud::value - 1;
            static inline constexpr uint16_t startPeriod = ((3 * period) / 2) - 50;
//            static inline constexpr uint16_t startPeriod = period / 2;
//                std::integral_constant<uint16_t, period>::_;
            
            struct StartBitHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Port<typename RxPin::name_type>> {
                static inline void isr() {
//                    Dbg::high();
                    RxPin::resetInt();
                    if (isRxEn()) {
                        RxPin::template attributes<rxPinAttrNoInt>();
                        data = 0x00_B;
                        setStartBit();
                        setReceiving();
                        *mcu_tcd()->cmpbclr = startPeriod;
                        waitFor<Status_t::enready>(status_r());
                        mcu_tcd()->ctrla.template set<CtrlA4_t::enable>();
                        waitFor<Status_t::cmdready>(status_r());
                        *mcu_tcd()->cmpbclr = period;
                        mcu_tcd()->ctrle.template set<CtrlE_t::synceoc>();
                    }
                }
            };
            struct BitHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Tcd<N>::Ovf> {
                static inline void isr() {
//                    Dbg::toggle();
                    mcu_tcd()->intflags.template reset<mcu_timer_t::IntFlags_t::ovf>();
                    if (isReceiving()) {
                        if (isStartBit()) {
                            if (RxPin::isHigh()) {
                                data = 0x80_B;
                            }
                            bitCount = 7;
                            clearStartBit();
                        }
                        else {
                            data = data >> 1;
                            if (RxPin::isHigh()) {
                                data = data | 0x80_B;
                            }
                            bitCount = bitCount - 1;
                            if (bitCount == 0) {
                                mcu_tcd()->ctrla.template clear<CtrlA4_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                                if constexpr (RecvQLength::value > 0) {
                                    static_assert(std::is_same_v<PA, External::Hal::NullProtocollAdapter>, "recvQueue is used, no need for PA");
                                    mRecvQueue.push_back(data);
                                }
                                else {
                                    static_assert(RecvQLength::value == 0);
                                    if constexpr(!std::is_same_v<PA, External::Hal::NullProtocollAdapter>) {
                                        if (!PA::process(std::byte{data})) {
                                            assert("input not handled by protocoll adapter");
                                        }
                                    }
                                    else {
                                        static_assert(std::false_t<MCU>::value, "no recvQueue, no PA -> wrong configuration");
                                    }
                                }
                                clearReceiving();
//                                Dbg::low();
                                RxPin::template attributes<rxPinAttrInt>();
                            }
                        }
                    }
                    if constexpr(!std::is_same_v<TxPin, void>) {
                        if (isSending()) {
//                            Dbg::toggle();
                            bitCount = bitCount - 1;
                            if (bitCount == 9) {
                                TxPin::low(); 
                            }   
                            else if (bitCount == 0) {
                                TxPin::high();
                                if (auto d = mSendQueue.pop_front()) {
                                    data = *d;
                                    bitCount = 10;
                                }
                                else {
                                    clearSending();
                                }
                            }
                            else {
//                                if (data & 0x01) {
                                if (std::any(data & 0x01_B)) {
                                    TxPin::high(); 
                                }
                                else {
                                    TxPin::low(); 
                                }
                                data = data >> 1;
                            }
                        }
                    }
                }
            };
            
            template<typename Mode>
            inline static void init() {
                static_assert(std::is_same_v<Mode, AVR::HalfDuplex>, "Only half-duplex possible");
                
                Dbg::template dir<AVR::Output>();
                
                if constexpr(!std::is_same_v<RxPin, void>) {
                    RxPin::template dir<AVR::Input>();
                }                
                if constexpr(!std::is_same_v<TxPin, void>) {
                    if constexpr(!std::is_same_v<TxPin, RxPin>) {
                        TxPin::template dir<AVR::Output>();
                        if constexpr(Inverted::value) {
                            TxPin::template attributes<Meta::List<AVR::Attributes::Inverting<>>>();
                        }
                    }
                }   
                mcu_tcd()->intflags.template reset<mcu_timer_t::IntFlags_t::ovf>();
                mcu_tcd()->intctrl.template set<mcu_timer_t::IntCtrl_t::ovf>();
                waitFor<Status_t::cmdready>(status_r());
                mcu_tcd()->ctrlc.template set<CtrlC_t::aUpdate>();
                
                mcu_tcd()->ctrla.template clear<mcu_timer_t::CtrlA4_t::enable>();
                clearBits();
                rxEnable<true>();
            }

            inline static void put(const std::byte b) {
                etl::Scoped<etl::DisbaleInterrupt<>> di;
                mSendQueue.push_back(b);
                if (!isSending()) {
                    startSending();
                }
            }
            
            template<bool B>
            inline static void rxEnable() {
                if constexpr(!std::is_same_v<RxPin, void>) {
                    etl::Scoped<etl::DisbaleInterrupt<>> di;
                    if constexpr(B) {
                        RxPin::resetInt();
                        RxPin::template attributes<rxPinAttrInt>();
                        if constexpr(std::is_same_v<RxPin, TxPin>) {
                            RxPin::template dir<AVR::Input>();
                        }
                        setRxEn();
                    }
                    else {
                        RxPin::template attributes<rxPinAttrNoInt>();
                        if constexpr(std::is_same_v<RxPin, TxPin>) {
                            TxPin::template dir<AVR::Output>();
                        }
                        clearRxEn();
                    }
                }
            }
            
            inline static bool isIdle() {
                etl::Scoped<etl::DisbaleInterrupt<>> di;
                return !isSending() && mSendQueue.empty();
            }
        private:
            inline static void startSending() {
                Dbg::high();
                if (!isReceiving()) {
                    if (auto d = mSendQueue.pop_front()) {
                        waitFor<Status_t::cmdready>(status_r());
                        *mcu_tcd()->cmpbclr = period;
                        mcu_tcd()->ctrle.template set<CtrlE_t::sync>();
                        waitFor<Status_t::enready>(status_r());
                        mcu_tcd()->ctrla.template set<CtrlA4_t::enable>();
                        bitCount = 10;
                        data = *d;
                        setSending();
                    }
                }
                Dbg::low();
            }
            
            inline static volatile etl::FiFo<std::byte, RecvQLength::value> mRecvQueue;
            inline static volatile etl::FiFo<std::byte, SendQLength::value> mSendQueue;
        };
        
        
        
        
        template<AVR::Concepts::Pin RxPin, AVR::Concepts::Pin TxPin, auto N, AVR::Concepts::ProtocolAdapter PA, 
                 etl::Concepts::NamedConstant Baud,
                 etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength, 
                 etl::Concepts::NamedFlag Inverted,
                 typename Dbg,                 
                 AVR::Concepts::At01Series MCU>
        struct Usart<Meta::List<RxPin, TxPin>, AVR::Component::Tca<N>, PA, Baud, 
                RecvQLength, SendQLength, Inverted, Dbg, MCU> final {
            
            using mcu_timer_t = typename MCU::TCA; 
            static constexpr auto mcu_tca = AVR::getBaseAddr<mcu_timer_t, N>;
        
            using gpior_t = typename MCU::Gpior; 
            static inline volatile uint8_t data;
            static inline /*volatile */ uint8_t bitCount;
            
            static inline constexpr auto flags = AVR::getBaseAddr<gpior_t, 0>;
            static inline constexpr uint8_t startMask = 0x01;
            static inline constexpr uint8_t stopMask = 0x02;
            static inline constexpr uint8_t errorMask = 0x03;
            
            static inline bool isStartBit() {
                return (flags()->data & startMask);
            }
            static inline void clearStartBit() {
                flags()->data = flags()->data & ~(startMask);
            }
            static inline void setStartBit() {
                flags()->data = flags()->data | startMask;
            }
            static inline bool isStopBit() {
                return (flags()->data & stopMask);
            }
            static inline void clearStopBit() {
                flags()->data = flags()->data & ~(stopMask);
            }
            static inline void setStopBit() {
                flags()->data = flags()->data | stopMask;
            }
            static inline bool isErrorBit() {
                return (flags()->data & errorMask);
            }
            static inline void clearErrorBit() {
                flags()->data = flags()->data & ~(errorMask);
            }
            static inline void setErrorBit() {
                flags()->data = flags()->data | errorMask;
            }
            static inline void clearBits() {
                flags()->data = 0x00;
            }
            
            static inline constexpr uint16_t period = Project::Config::fMcu.value / Baud::value;
            static inline constexpr uint16_t startPeriod = (3 * period) / 2;
//            static inline constexpr uint16_t startPeriod = period / 2;
//                std::integral_constant<uint16_t, period>::_;
            
            struct StartBitHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Port<typename RxPin::name_type>> {
                static inline void isr() {
                    RxPin::resetInt();
                    RxPin::template attributes<Meta::List<AVR::Attributes::Pullup<MCU>>>();
                    setStartBit();
                    *mcu_tca()->cnt = 0;
                    *mcu_tca()->per = startPeriod;
                    data = 0x00;
                    mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA_t::enable>();
                }
            };
            struct BitHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Tca<N>::Ovf> {
                static inline void isr() {
                    mcu_tca()->intflags.template reset<mcu_timer_t::Intflags_t::ovf>();
#if 1
                    if (isStartBit()) {
                        *mcu_tca()->per = period;
                        if (RxPin::isHigh()) {
                            data = 0x80;
                        }
                        bitCount = 7;
                        clearStartBit();
                    }
                    else {
                        data = data >> 1;
                        if (RxPin::isHigh()) {
                            data = data | 0x80;
                        }
                        bitCount = bitCount - 1;
                        if (!bitCount) {
                            mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                            RxPin::template attributes<Meta::List<AVR::Attributes::Pullup<MCU>, AVR::Attributes::Interrupt<AVR::Attributes::OnFalling>>>();
                            PA::process(std::byte{data});
                            clearBits();
                        }
                    }
#endif
#if 0
                    if (isStartBit()) {
                        mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                        *mcu_tca()->per = period;
                        mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA_t::enable>();
                        bitCount = 0;
                        clearStartBit();
                    }
                    else {
                        if (isStopBit()) {
                            mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                            RxPin::template attributes<Meta::List<AVR::Attributes::Pullup<MCU>, AVR::Attributes::Interrupt<AVR::Attributes::OnFalling>>>();
                            PA::process(std::byte{data});
                            data = 0x00;
                            clearBits();
                        }
                        else {
                            data = (data >> 1) & 0xff;
                            if (RxPin::isHigh()) {
                                data = data | 0x80;
                            }
                            bitCount = bitCount + 1;
                            if (bitCount == 8) {
                                setStopBit();
                            }
                        }
                    }
#endif
                }
            };
            inline static void init() {
                if constexpr(!std::is_same_v<RxPin, void>) {
                    RxPin::template dir<AVR::Input>();
                    RxPin::template attributes<Meta::List<AVR::Attributes::Pullup<MCU>, AVR::Attributes::Interrupt<AVR::Attributes::OnFalling>>>();
                }                
                if constexpr(!std::is_same_v<TxPin, void>) {
                }   
                mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA_t::enable>();
                mcu_tca()->intctrl.template set<mcu_timer_t::Intctrl_t::ovf>();
                clearBits();
            }
        };
    }
}
