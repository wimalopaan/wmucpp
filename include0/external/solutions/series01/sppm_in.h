#pragma once

#include <std/chrono>

#include "mcu/common/concepts.h"

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

namespace External {
    using namespace std::literals::chrono;
    
    namespace Ppm {
        
        template<typename SPpmIn>
        struct Adapter {
            struct Protocoll_Adapter {
                using value_type = SPpmIn::value_type;
                
                static inline value_type value(uint8_t) {
                    return SPpmIn::value();
                }
                
                static inline uint16_t packages() {
                    return SPpmIn::counter();    
                }
                static inline void resetStats() {
                    SPpmIn::reset();    
                }
                static inline void ratePeriodic() {}
                
            };  
            using protocoll_adapter_type = Protocoll_Adapter;
            
            inline static void periodic() {
                SPpmIn::periodic();
            }

            static inline void put(std::byte) {}
            
            static inline std::byte get() {
                return std::byte{};
            }
            
            static inline void init() {
                SPpmIn::init();
            }
        };
        
        template<typename TimerNumber, typename ClockP = void, typename MCU = DefaultMcuType>
        struct SinglePpmIn;
        
        template<uint8_t N, typename ClockP, AVR::Concepts::At01DxSeries MCU>
        struct SinglePpmIn<AVR::Component::Tcb<N>, ClockP, MCU> {
            static inline constexpr auto mcu_tcb = AVR::getBaseAddr<typename MCU::TCB, N>;
            
            using ctrla_t = MCU::TCB::CtrlA_t;
            using ctrlb_t = MCU::TCB::CtrlB_t;
            using intflags_t = MCU::TCB::IntFlags_t;
            using ev_t = MCU::TCB::EvCtrl_t;
            
            using clock_provider = ClockP;
            
//            inline static constexpr uint8_t prescaler = 2;
            inline static constexpr uint16_t prescaler = []{
                if constexpr(std::is_same_v<clock_provider, void>) {
                    return 2;
                }
                else if constexpr(std::is_same_v<typename clock_provider::component_type, AVR::Component::Tca<0>>) {
                    return clock_provider::prescaler_type::value;
                }
            }();
            
            inline static constexpr uint16_t ppmMaxExtended = 2250_us * (Project::Config::fMcu / prescaler);
            inline static constexpr uint16_t ppmMax = 2_ms * (Project::Config::fMcu / prescaler);
            inline static constexpr uint16_t ppmMin = 1_ms * (Project::Config::fMcu / prescaler);
            inline static constexpr uint16_t ppmMinExtended = 750_us * (Project::Config::fMcu / prescaler);
            
            static inline constexpr uint16_t ppmWidth = ppmMax - ppmMin;
            static inline constexpr uint16_t span = ppmWidth / 2;
            static inline constexpr uint16_t medium = (ppmMax + ppmMin) / 2;
            
            using value_type = etl::uint_ranged_NaN<uint16_t, ppmMin, ppmMax>;
            using extended_value_type = etl::uint_ranged_NaN<uint16_t, ppmMinExtended, ppmMaxExtended>;

            inline static constexpr int16_t sppmMax = ppmMax;
            inline static constexpr int16_t sppmMin = ppmMin;
            inline static constexpr int16_t smedium = medium;
            inline static constexpr int16_t sspan   = span;
            inline static constexpr int16_t sppmWidth = ppmWidth;
            inline static constexpr int16_t sppmMinExtended = ppmMinExtended;
            inline static constexpr int16_t sppmMaxExtended = ppmMaxExtended;
            
            using svalue_type = etl::int_ranged_NaN<int16_t, sppmMin - smedium, sppmMax - smedium>;
//            using extended_svalue_type = etl::int_ranged_NaN<int16_t, (sppmMinExtended - smedium), (sppmMaxExtended - smedium)>;
            
//            std::integral_constant<decltype(ppmMax), ppmMax>::_;
//            std::integral_constant<decltype(ppmMin), ppmMin>::_;
//            std::integral_constant<decltype(ppmMaxExtended), ppmMaxExtended>::_;
//            std::integral_constant<decltype(ppmMinExtended), ppmMinExtended>::_;

//            std::integral_constant<decltype(sppmMaxExtended), (sppmMaxExtended - smedium)>::_;
//            std::integral_constant<decltype(sppmMaxExtended), sppmMinExtended - smedium>::_;
            
            inline static void init() {
                mcu_tcb()->ctrlb.template set<ctrlb_t::mode_pw>();
                mcu_tcb()->evctrl.template set<ev_t::captei>();
                mcu_tcb()->ctrla.template set<ctrla_t::clkdiv2 | ctrla_t::enable>();
            }
            
            inline static uint16_t raw() {
                return *mcu_tcb()->ccmp;
            }
            
            inline static void periodic() {
                onCapture([]{
                    const auto v = *mcu_tcb()->ccmp;
                    if ((v >= ppmMinExtended) && (v <ppmMaxExtended)) {
                        ++mCounter;
                        mValue = std::clamp(v, ppmMin, ppmMax);
                    }
                });                
            }

            inline static void onCapture(auto f) {
                mcu_tcb()->intflags.template testAndReset<intflags_t::capt>(f);
            }
            
            inline static value_type value() {
                const auto v = *mcu_tcb()->ccmp;
                if ((v >= ppmMinExtended) && (v <ppmMaxExtended)) {
                    return {std::clamp(v, ppmMin, ppmMax)};
                }
                return {};
            }
            inline static svalue_type svalue() {
                return svalue_type{value().toInt() - smedium};
            }

            inline static extended_value_type extended() {
                const auto v = *mcu_tcb()->ccmp;
                if ((v >= ppmMinExtended) && (v <ppmMaxExtended)) {
                    return {v};
                }
                return extended_value_type{};
            }
            
            inline static auto counter() {
                return mCounter;
            }
            inline static void reset() {
                mCounter.setToBottom();
                mValue.setNaN();
            }
        private:
            static inline value_type mValue{};
            static inline etl::uint_ranged<uint16_t> mCounter{};
        };
    }
}
