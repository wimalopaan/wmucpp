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
        
        template<typename TN, typename Prescaler = std::integral_constant<uint8_t, 8>, 
                 etl::Concepts::NamedFlag UseIsr = etl::NamedFlag<false>, typename MCU = DefaultMcuType>
        struct Cppm;
        
        template<auto TN, typename Pos, typename Pre, typename UseIsr, AVR::Concepts::At01DxSeries MCU>
        struct Cppm<AVR::Portmux::Position<AVR::Component::Tca<TN>, Pos>, Pre, UseIsr, MCU> {
            inline static constexpr bool useIsr = UseIsr::value;
            using mcu_timer_t = typename MCU::TCA; 
            static constexpr auto mcu_tca = AVR::getBaseAddr<mcu_timer_t, TN>;
            using position = AVR::Portmux::Position<AVR::Component::Tca<TN>, Pos>;

            static inline constexpr uint16_t prescaler = Pre::value;
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
        
            static constexpr uint16_t ccPulse = 500_us * timerFrequency;
            static_assert(ccPulse > 0, "wrong oc value");
            static constexpr uint16_t ccPulseFrame = ocMax + ccPulse;
            static_assert(ccPulse > 0, "wrong oc value");
            
            using ranged_type   = etl::uint_ranged<uint16_t, ocMin, ocMax> ;
            
            struct Frame {
                volatile uint16_t pulse{ocMedium};
            };
            
            using channel_t = etl::uint_ranged<uint8_t, 0, 7>;
            
            using frames_t = std::array<Frame, channel_t::Upper + 2>;
            
            using ca_t = mcu_timer_t::CtrlA_t;
            using cb_t = mcu_timer_t::CtrlB_t;
            using ic_t = mcu_timer_t::Intctrl_t;

            template<typename CB>
            struct CmpHandler : AVR::IsrBaseHandler<AVR::ISR::Tca<0>::Cmp<1>> {
                static inline void isr() {
                    CB::once();
                    mcu_tca()->intflags.template reset<mcu_timer_t::Intflags_t::cmp1>();
                    ++mIndex;
                    *mcu_tca()->perbuf = mFrames[mIndex].pulse;
                    *mcu_tca()->cmp1buf = ccPulse;
                }    
            };
            
            template<typename CB>
            struct OvfHandler : AVR::IsrBaseHandler<AVR::ISR::Tca<0>::Ovf> {
                static inline void isr() {
                    CB::once();
                    mcu_tca()->intflags.template reset<mcu_timer_t::Intflags_t::ovf>();
                }    
            };
            
            
            static inline constexpr auto pv = []{
                for(auto p : mcu_timer_t::prescalerValues) {
                    if (p.scale == prescaler) {
                        return p.bits;
                    }
                }
            }();
            
            static inline void init() {
                if constexpr(useIsr) {
                    mcu_tca()->intctrl.template set<ic_t::ovf | ic_t::cmp1>();
                }
                mcu_tca()->ctrla.template set<pv | ca_t::enable>();
                mcu_tca()->ctrlb.template set<cb_t::cmp1en | cb_t::pwm>();
                *mcu_tca()->perbuf = ocMedium;
                *mcu_tca()->cmp1buf = ccPulse;
                mFrames[8].pulse = ocFrame - (8 * ocMedium);
            }

            template<bool use = useIsr, typename = std::enable_if_not_t<use>> 
            static inline void periodic() {
                mcu_tca()->intflags.template testAndReset<mcu_timer_t::Intflags_t::cmp1>([]{
                    ++mIndex;
                    *mcu_tca()->perbuf = mFrames[mIndex].pulse;
                    *mcu_tca()->cmp1buf = ccPulse;
                });
            }

            template<bool use = useIsr, typename = std::enable_if_not_t<use>> 
            static inline void periodic(auto f) {
                mcu_tca()->intflags.template testAndReset<mcu_timer_t::Intflags_t::cmp1>([&]{
                    ++mIndex;
                    *mcu_tca()->perbuf = mFrames[mIndex].pulse;
                    *mcu_tca()->cmp1buf = ccPulse;
                    f();
                });
            }

            template<bool use = useIsr, typename = std::enable_if_not_t<use>> 
            static inline void onOvfl(auto f) {
                mcu_tca()->intflags.template testAndReset<mcu_timer_t::Intflags_t::ovf>([&]{
                    f();
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
