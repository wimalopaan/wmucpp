#pragma once

#include <std/chrono>

#include "mcu/common/concepts.h"
#include "mcu/internals/port.h"
#include "mcu/common/isr.h"

#include <external/units/physical.h>

namespace External {
    namespace Rpm {
        template<typename ClockTimer, typename EvTimer, typename MCU = DefaultMcuType>
        struct RpmFreq;
        template<auto Na, auto Nb, AVR::Concepts::At01Series MCU>
        struct RpmFreq<AVR::Component::Tca<Na>, AVR::Component::Tcb<Nb>, MCU> {
            static inline constexpr auto mcu_tca = AVR::getBaseAddr<typename MCU::TCA, Na>;
            static inline constexpr auto mcu_tcb = AVR::getBaseAddr<typename MCU::TCB, Nb>;
            
            using Actrla_t = MCU::TCA::CtrlA_t;
            
            using Bctrla_t = MCU::TCB::CtrlA_t;
            using Bctrlb_t = MCU::TCB::CtrlB_t;
            using Bev_t = MCU::TCB::EvCtrl_t;
            using Int_t = MCU::TCB::IntFlags_t;
            
            inline static constexpr uint16_t prescaler = 1024;
            inline static constexpr auto fTimer = Project::Config::fMcu / prescaler;
            inline static constexpr auto rpm = fTimer.value * 60;
            inline static constexpr auto cmin = rpm / std::numeric_limits<uint16_t>::max();

//            std::integral_constant<decltype(cmin), cmin>::_;
//            std::integral_constant<decltype(rpm), rpm>::_;
//            decltype(fTimer)::_;
            
            inline static constexpr auto pva = []{
                for(const auto pv : MCU::TCA::prescalerValues) {
                    if (pv.scale == prescaler) {
                        return pv.bits;
                    }
                }
            }();
            
//            std::integral_constant<decltype(pva), pva>::_;
            
            inline static void init() {
                mcu_tca()->ctrla.template set<Actrla_t::enable | pva>();

                mcu_tcb()->ctrlb.template set<Bctrlb_t::mode_frq>();
                mcu_tcb()->evctrl.template set<Bev_t::captei>();
                mcu_tcb()->ctrla.template set<Bctrla_t::clktca | Bctrla_t::enable>();
            }
            
            inline static External::Units::RPM last;
            
            inline static void reset() {
                last = External::Units::RPM{};
            }
            
            inline static External::Units::RPM value() {
                if (mcu_tcb()->intflags.template isSet<Int_t::capt>()) {
                    auto c = *mcu_tcb()->ccmp;
//                    last = External::Units::RPM{fTimer / c};
                    last = External::Units::RPM(rpm / c);
                }
                return External::Units::RPM{last};    
            }
        };
        
        
        template<typename Pin, typename Timer, typename MCU = DefaultMcuType>
        struct RpmGpio {
            using port = Pin::port;
            using port_name = port::name_type;
            
            using isr =  typename AVR::ISR::Port<port_name>;

            inline static void init() {
                Pin::template dir<AVR::Input>();
                Pin::template attributes<Meta::List<AVR::Attributes::Interrupt<AVR::Attributes::OnRising>>>();
                Pin::template pullup<true>();
            }
            
            struct ImpulsIsr : public AVR::IsrBaseHandler<isr> {
                inline static void isr() {
                    Pin::resetInt();
                    auto value = Timer::actual();
                    mDiff = value - mLastValue;
                    mLastValue = value;
                }
            };
            
            inline static const auto& diff() {
                return mDiff;
            }
            
        private:
            inline static uint16_t mLastValue = 0;
            inline static uint16_t mDiff = 0;
        };
    }
    
}
