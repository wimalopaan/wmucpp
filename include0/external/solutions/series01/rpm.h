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

        template<auto Na, auto Nb, AVR::Concepts::AtDxSeries MCU>
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
                    const auto c = *mcu_tcb()->ccmp;
//                    last = External::Units::RPM{fTimer / c};
                    last = External::Units::RPM(rpm / c);
                }
                return External::Units::RPM{last};    
            }
        };
        
        
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
                    const auto c = *mcu_tcb()->ccmp;
//                    last = External::Units::RPM{fTimer / c};
                    last = External::Units::RPM(rpm / c);
                }
                return External::Units::RPM{last};    
            }
        };

        template<auto Nb, AVR::Concepts::At01Series MCU>
        struct RpmFreq<void, AVR::Component::Tcb<Nb>, MCU> {
            static inline constexpr auto mcu_tcb = AVR::getBaseAddr<typename MCU::TCB, Nb>;
            
            using Bctrla_t = MCU::TCB::CtrlA_t;
            using Bctrlb_t = MCU::TCB::CtrlB_t;
            using Bev_t = MCU::TCB::EvCtrl_t;
            using Int_t = MCU::TCB::IntFlags_t;
            
            inline static constexpr uint16_t prescaler = 256;
//            inline static constexpr uint32_t prescaler = 65536;
            inline static constexpr auto fTimer = Project::Config::fMcu / prescaler;
            inline static constexpr uint32_t rpm = fTimer.value * 60;
            inline static constexpr auto cmin = rpm / std::numeric_limits<uint16_t>::max();

//            std::integral_constant<decltype(cmin), cmin>::_;
//            std::integral_constant<decltype(rpm), rpm>::_;
//            decltype(fTimer)::_;
            
            inline static void init() {
                mcu_tcb()->ctrlb.template set<Bctrlb_t::mode_frq>();
                mcu_tcb()->evctrl.template set<Bev_t::captei>();
                mcu_tcb()->ctrla.template set<Bctrla_t::clktca | Bctrla_t::enable>();
            }
            
            inline static void reset() {
                if (!mImpuls) {
                    last = External::Units::RPM{};
                }
                else {
                    mImpuls = false;
                }
            }
            
            inline static void periodic() {
                if (mcu_tcb()->intflags.template isSet<Int_t::capt>()) {
                    mImpuls = true;
                    const uint16_t c = *mcu_tcb()->ccmp; // reset flag
                    if (c < mLastCounter) {
                        ++mOvrFlow;
                    }
                    const uint16_t v = (mOvrFlow << 8) | ((c >> 8) & 0xff);  
                    if (v > 0) {
                        last = External::Units::RPM(rpm / v);
                    }
                    mOvrFlow = 0;
                }
                else {
                    const uint16_t cv = *mcu_tcb()->cnt;
                    if (cv < mLastCounter) {
                        ++mOvrFlow;
                    }
                }
                mLastCounter = *mcu_tcb()->cnt;                
            }
            inline static External::Units::RPM value() {
                return External::Units::RPM{last};    
            }
            inline static uint16_t counter() {
                return mLastCounter;
            }
            inline static uint8_t overflows() {
                return mOvrFlow;
            }
        private:
            inline static External::Units::RPM last;            
            inline static bool mImpuls{false};
            inline static uint16_t mLastCounter{0};
            inline static uint8_t mOvrFlow{0};
//            inline static uint8_t mOvrFlowLast{0};
            
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
