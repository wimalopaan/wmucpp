#pragma once

#include <std/chrono>

#include "mcu/common/concepts.h"

namespace External {
    using namespace std::literals::chrono;
    
    namespace Ppm {
        template<typename TimerNumber, typename MCU = DefaultMcuType>
        struct SinglePpmIn;
        
        template<uint8_t N, AVR::Concepts::At01Series MCU>
        struct SinglePpmIn<AVR::Component::Tcb<N>, MCU> {
            static inline constexpr auto mcu_tcb = AVR::getBaseAddr<typename MCU::TCB, N>;
            
            using ctrla_t = MCU::TCB::CtrlA_t;
            using ctrlb_t = MCU::TCB::CtrlB_t;
            using ev_t = MCU::TCB::EvCtrl_t;
            
            inline static void init() {
                mcu_tcb()->ctrlb.template set<ctrlb_t::mode_pw>();
                mcu_tcb()->evctrl.template set<ev_t::captei>();
                mcu_tcb()->ctrla.template set<ctrla_t::clkdiv2 | ctrla_t::enable>();
            }
            
            inline static uint16_t value() {
                return *mcu_tcb()->ccmp;
            }

            inline static uint16_t counter() {
                return *mcu_tcb()->cnt;
            }
            
        };
    }
}
