#pragma once

#include <std/chrono>

#include "mcu/common/concepts.h"

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

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
            using intflags_t = MCU::TCB::IntFlags_t;
            using ev_t = MCU::TCB::EvCtrl_t;
            
            inline static constexpr uint8_t prescaler = 2;
            
            inline static constexpr uint16_t ppmMaxExtended = 2250_us * (Project::Config::fMcu / prescaler);
            inline static constexpr uint16_t ppmMax = 2_ms * (Project::Config::fMcu / prescaler);
            inline static constexpr uint16_t ppmMin = 1_ms * (Project::Config::fMcu / prescaler);
            inline static constexpr uint16_t ppmMinExtended = 750_us * (Project::Config::fMcu / prescaler);
            
            static inline constexpr uint16_t ppmWidth = ppmMax - ppmMin;
            static inline constexpr uint16_t span = ppmWidth / 2;
            static inline constexpr uint16_t medium = (ppmMax + ppmMin) / 2;
            
            using value_type = etl::uint_ranged_NaN<uint16_t, ppmMin, ppmMax>;
            using extended_value_type = etl::uint_ranged_NaN<uint16_t, ppmMinExtended, ppmMaxExtended>;
            
//            std::integral_constant<decltype(ppmMax), ppmMax>::_;
//            std::integral_constant<decltype(ppmMin), ppmMin>::_;
//            std::integral_constant<decltype(ppmMaxExtended), ppmMaxExtended>::_;
//            std::integral_constant<decltype(ppmMinExtended), ppmMinExtended>::_;
            
            inline static void init() {
                mcu_tcb()->ctrlb.template set<ctrlb_t::mode_pw>();
                mcu_tcb()->evctrl.template set<ev_t::captei>();
                mcu_tcb()->ctrla.template set<ctrla_t::clkdiv2 | ctrla_t::enable>();
            }
            
            inline static uint16_t raw() {
                return *mcu_tcb()->ccmp;
            }

            inline static void onCapture(auto f) {
                mcu_tcb()->intflags.template testAndReset<intflags_t::capt>(f);
            }
            
            inline static value_type value() {
                auto v = *mcu_tcb()->ccmp;
                if ((v >= ppmMinExtended) && (v <ppmMaxExtended)) {
                    v = std::clamp(v, ppmMin, ppmMax);
                    return {v};
                }
                return value_type{};
            }

            inline static extended_value_type extended() {
                auto v = *mcu_tcb()->ccmp;
                if ((v >= ppmMinExtended) && (v <ppmMaxExtended)) {
                    return {v};
                }
                return extended_value_type{};
            }

//            inline static Hott::hott_t hott() {
//                return Hott::hott_t((uint32_t(*mcu_tcb()->ccmp - ppmMin) * (Hott::hott_t::Upper - Hott::hott_t::Lower)) / (ppmMax - ppmMin) + Hott::hott_t::Lower);
//            }
        };
    }
}
