#pragma once

#include <cstdint>

#include <mcu/common/isr.h>
#include <mcu/common/concepts.h>

namespace External {
    namespace SoftSerial {

        template<typename Pins, typename Timer, AVR::Concepts::ProtocolAdapter PA = External::Hal::NullProtocollAdapter,
                 etl::Concepts::NamedConstant Baud = AVR::BaudRate<9600>,
                 etl::Concepts::NamedConstant RecvQLength = AVR::ReceiveQueueLength<64>, 
                 etl::Concepts::NamedConstant SendQLength = AVR::SendQueueLength<64>, typename MCU = DefaultMcuType> 
        struct Usart;

        template<AVR::Concepts::Pin RxPin, AVR::Concepts::Pin TxPin, auto N, AVR::Concepts::ProtocolAdapter PA, 
                 etl::Concepts::NamedConstant Baud,
                 etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength, AVR::Concepts::At01Series MCU>
        struct Usart<Meta::List<RxPin, TxPin>, AVR::Component::Tcd<N>, PA, Baud, RecvQLength, SendQLength, MCU> final {
            
            using mcu_timer_t = typename MCU::TCD; 
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
//                    *mcu_tca()->cnt = 0;
                    *mcu_tca()->cmpbclr = startPeriod;
                    data = 0x00;
                    mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA4_t::enable>();
                }
            };
            struct RxBitHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Tcd<N>::Ovf> {
                static inline void isr() {
                    mcu_tca()->intflags.template reset<mcu_timer_t::IntFlags_t::ovf>();
                    if (isStartBit()) {
                        mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA4_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                        *mcu_tca()->cmpbclr = period;
                        mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA4_t::enable>();
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
                            mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA4_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                            RxPin::template attributes<Meta::List<AVR::Attributes::Pullup<MCU>, AVR::Attributes::Interrupt<AVR::Attributes::OnFalling>>>();
                            PA::process(std::byte{data});
                            clearBits();
                        }
                    }
                }
            };
            inline static void init() {
                if constexpr(!std::is_same_v<RxPin, void>) {
                    RxPin::template dir<AVR::Input>();
                    RxPin::template attributes<Meta::List<AVR::Attributes::Pullup<MCU>, AVR::Attributes::Interrupt<AVR::Attributes::OnFalling>>>();
                }                
                if constexpr(!std::is_same_v<TxPin, void>) {
                }   
                mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA4_t::enable>();
                mcu_tca()->intctrl.template set<mcu_timer_t::IntCtrl_t::ovf>();
                clearBits();
            }
        };
        
        
        
        
        template<AVR::Concepts::Pin RxPin, AVR::Concepts::Pin TxPin, auto N, AVR::Concepts::ProtocolAdapter PA, 
                 etl::Concepts::NamedConstant Baud,
                 etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength, AVR::Concepts::At01Series MCU>
        struct Usart<Meta::List<RxPin, TxPin>, AVR::Component::Tca<N>, PA, Baud, RecvQLength, SendQLength, MCU> final {
            
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
            struct RxBitHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Tca<N>::Ovf> {
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
