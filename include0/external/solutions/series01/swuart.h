#pragma once

#include <cstdint>

#include <mcu/common/isr.h>
#include <mcu/common/concepts.h>

namespace External {
    namespace SoftSerial {

        template<typename Pins, typename Timer, AVR::Concepts::ProtocolAdapter PA = External::Hal::NullProtocollAdapter,
                 etl::Concepts::NamedConstant Baud = AVR::BaudRate<9600>,
                 etl::Concepts::NamedConstant RecvQLength = ReceiveQueueLength<64>, 
                 etl::Concepts::NamedConstant SendQLength = SendQueueLength<64>, typename MCU = DefaultMcuType> 
        struct Usart;

        template<AVR::Concepts::Pin RxPin, AVR::Concepts::Pin TxPin, auto N, AVR::Concepts::ProtocolAdapter PA, 
                 etl::Concepts::NamedConstant Baud,
                 etl::Concepts::NamedConstant RecvQLength, etl::Concepts::NamedConstant SendQLength, AVR::Concepts::At01Series MCU>
        struct Usart<Meta::List<RxPin, TxPin>, AVR::Component::Tca<N>, PA, Baud, RecvQLength, SendQLength, MCU> final {
            
            using mcu_timer_t = typename MCU::TCA; 
            static constexpr auto mcu_tca = getBaseAddr<mcu_timer_t, N>;
        
            using gpior_t = typename MCU::Gpior; 
            static inline volatile uint8_t data;
            static inline volatile uint8_t bitCount;
            
            static inline bool isStartBit() {
                static constexpr auto flags = getBaseAddr<gpior_t, 0>;
                return (flags()->data & 0x01);
            }
            static inline void clearStartBit() {
                static constexpr auto flags = getBaseAddr<gpior_t, 0>;
                flags()->data = flags()->data & ~(0x01);
            }
            static inline void setStartBit() {
                static constexpr auto flags = getBaseAddr<gpior_t, 0>;
                flags()->data = flags()->data | 0x01;
            }
            
            static inline constexpr uint16_t period = Project::Config::fMcu.value / Baud::value;
            static inline constexpr uint16_t startPeriod = period / 2;
//                std::integral_constant<uint16_t, period>::_;
            
            struct StartBitHandler : public IsrBaseHandler<typename AVR::ISR::Port<typename RxPin::name_type>> {
                static inline void isr() {
                    *mcu_tca()->cnt = 0;
                    *mcu_tca()->per = startPeriod;
                    mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA_t::enable>();
                    setStartBit();
                    RxPin::template attributes<Meta::List<Attributes::Pullup<MCU>>>();
                }
            };
            struct RxBitHandler : public IsrBaseHandler<typename AVR::ISR::Tca<0>::Ovf> {
                static inline void isr() {
                    if (isStartBit()) {
                        mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                        *mcu_tca()->cnt = 0;
                        *mcu_tca()->per = period;
                        mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA_t::enable>();
                        bitCount = 0;
                    }
                    else {
                        data = (data >> 1) & 0xff;
                        if (RxPin::isHigh()) {
                            data = data | 0x80;
                        }
                        bitCount = bitCount + 1;
                        if (bitCount == 8) {
                            mcu_tca()->ctrla.template clear<mcu_timer_t::CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                            RxPin::template attributes<Meta::List<Attributes::Pullup<MCU>, Attributes::Interrupt<Attributes::OnFalling>>>();
                            
                            PA::process(std::byte{data});
                        }
                    }
                }
            };
            inline static void init() {
                if constexpr(!std::is_same_v<RxPin, void>) {
                    RxPin::template dir<Input>();
                    RxPin::template attributes<Meta::List<Attributes::Pullup<MCU>, Attributes::Interrupt<Attributes::OnFalling>>>();
                }                
                if constexpr(!std::is_same_v<TxPin, void>) {
                }   
                mcu_tca()->intctrl.template set<mcu_timer_t::Intctrl_t::ovf>();
                clearStartBit();
            }
        };
    }
}
