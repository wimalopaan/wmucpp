#pragma once

#include <std/chrono>
#include <etl/rational.h>
#include <etl/ranged.h>

#include <mcu/common/ppm.h>

namespace External {
    namespace Ppm {
        using namespace External::Units;
        using namespace External::Units::literals;
        using namespace std::literals::chrono;
        
        template<typename TN, typename MCU = DefaultMcuType>
        struct Cppm;
        
        template<auto TN, typename Pos, AVR::Concepts::At01Series MCU>
        struct Cppm<AVR::Portmux::Position<AVR::Component::Tca<TN>, Pos>, MCU> {
            using mcu_timer_t = typename MCU::TCA; 
            static constexpr auto mcu_tca = AVR::getBaseAddr<mcu_timer_t, TN>;
            using position = AVR::Portmux::Position<AVR::Component::Tca<TN>, Pos>;

            static inline constexpr uint16_t prescaler = 8;
            static inline constexpr uint16_t period = (Project::Config::fMcu / prescaler) / 50_Hz;
//            std::integral_constant<uint16_t, period>::_;

            static constexpr hertz timerFrequency = Project::Config::fMcu / prescaler;
            static constexpr uint16_t ocMin = 1_ms * timerFrequency;
            static_assert(ocMin > 0, "wrong oc value");
            static constexpr uint16_t ocMax = 2_ms * timerFrequency;
            static_assert(ocMax > 0, "wrong oc value");
            static constexpr uint16_t ocDelta = ocMax - ocMin;
            static_assert(ocDelta > 0, "wrong oc value");
            static constexpr uint16_t ocFrame = 20_ms * timerFrequency;
            static_assert(ocFrame > 0, "wrong oc value");
            static constexpr uint16_t ocMedium = (ocMin + ocMax) / 2;
            static_assert(ocMedium> 0, "wrong oc value");
        
            static constexpr uint16_t ccPulse = 300_us * timerFrequency;
            static_assert(ccPulse > 0, "wrong oc value");
            static constexpr uint16_t ccPulseFrame = ocMax + ccPulse;
            static_assert(ccPulse > 0, "wrong oc value");
            
            using ranged_type   = etl::uint_ranged<uint16_t, ocMin, ocMax> ;
            
            struct Frame {
                uint16_t pulse{ocMedium};
            };
            
            using channel_t = etl::uint_ranged<uint8_t, 0, 7>;
            
            using frames_t = std::array<Frame, channel_t::Upper + 2>;
            
            using ca_t = mcu_timer_t::CtrlA_t;
            using cb_t = mcu_timer_t::CtrlB_t;
            
            static inline constexpr auto pv = []{
                for(auto p : mcu_timer_t::prescalerValues) {
                    if (p.scale == prescaler) {
                        return p.bits;
                    }
                }
            }();
            
            static inline void init() {
                mcu_tca()->ctrla.template set<pv | ca_t::enable>();
                mcu_tca()->ctrlb.template set<cb_t::cmp1en | cb_t::pwm>();
                *mcu_tca()->perbuf = ocMedium;
                *mcu_tca()->cmp1buf = ccPulse;
                mFrames[8].pulse = ocFrame - (8 * ocMedium);
            }
            static inline void periodic() {
                mcu_tca()->intflags.template testAndReset<mcu_timer_t::Intflags_t::cmp1>([]{
                    ++mIndex;
                    *mcu_tca()->perbuf = mFrames[mIndex].pulse;
                    *mcu_tca()->cmp1buf = ccPulse;
                });
            }
            static inline void set(const channel_t ch, const ranged_type& v) {
                mFrames[ch].pulse = v;    
                uint16_t sum = ocFrame;
                for(uint8_t i = 0; i < (frames_t::size() - 1); ++i) {
                    sum -= mFrames[i].pulse;
                }
                mFrames[frames_t::size() - 1].pulse = sum;
            }
        private:
            using index_t = etl::cyclic_type_t<frames_t>;
            static inline frames_t mFrames{};
            static inline index_t mIndex;
        };
    }
}
