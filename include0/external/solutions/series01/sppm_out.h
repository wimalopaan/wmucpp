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
        
        template<typename TN, typename ChannelList = Meta::List<>, typename MCU = DefaultMcuType>
        struct PpmOut;
        
        template<auto TN, typename Pos, typename... Channels,  AVR::Concepts::At01Series MCU>
        struct PpmOut<AVR::Portmux::Position<AVR::Component::Tca<TN>, Pos>, Meta::List<Channels...>, MCU> {
            using mcu_timer_t = typename MCU::TCA; 
            static constexpr auto mcu_tca = AVR::getBaseAddr<mcu_timer_t, TN>;
            using position = AVR::Portmux::Position<AVR::Component::Tca<TN>, Pos>;

            using index_type = etl::uint_ranged<uint8_t, 0, sizeof...(Channels)>;

            template<typename Position, uint8_t N>
            struct WOMapper;
            
            template<typename Position>
            struct WOMapper<Position, 0> {
                using pin = typename AVR::Portmux::Map<Position>::wo0pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::cmp0en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position>
            struct WOMapper<Position, 1> {
                using pin = typename AVR::Portmux::Map<Position>::wo1pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::cmp1en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position>
            struct WOMapper<Position, 2> {
                using pin = typename AVR::Portmux::Map<Position>::wo2pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::cmp2en;
                using type = std::integral_constant<decltype(value), value>;
            };
            
            template<typename WO>
            using womapper = WOMapper<position, WO::value>;
            

            template<typename WO>
            struct pinmapper {
                using type = typename WOMapper<position, WO::value>::pin;
            };
            
            using flag_list = Meta::transform_type<womapper, Meta::List<Channels...>>;
            using pin_list  = Meta::transform_type<pinmapper, Meta::List<Channels...>>;
            
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
        
            static constexpr uint16_t ccPulse = 500_us * timerFrequency;
            static_assert(ccPulse > 0, "wrong oc value");
            static constexpr uint16_t ccPulseFrame = ocMax + ccPulse;
            static_assert(ccPulse > 0, "wrong oc value");
            
//            using ranged_type   = etl::uint_ranged_NaN<uint16_t, ocMin, ocMax> ;
            using ranged_type   = etl::uint_ranged<uint16_t, ocMin, ocMax> ;
//            ranged_type::_;
                //            using extended_type = etl::uint_ranged_NaN<uint16_t, ocMin, ocMax> ;
            
            static inline constexpr auto pv = []{
                for(auto p : mcu_timer_t::prescalerValues) {
                    if (p.scale == prescaler) {
                        return p.bits;
                    }
                }
            }();
            
            template<bool start = true>
            static inline void init() {
                mcu_tca()->ctrla.template set<pv | mcu_timer_t::CtrlA_t::enable>();
                mcu_tca()->ctrlb.template set<mcu_timer_t::CtrlB_t::pwm>();
                constexpr auto value = Meta::value_or_v<flag_list>;
//                std::integral_constant<decltype(value), value>::_;
                if constexpr(start) {
                    mcu_tca()->ctrlb.template add<value, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                    AVR::PinGroup<pin_list>::template dir<AVR::Output>();
                }
                *mcu_tca()->perbuf = period;
            }
            
            static inline void on() {
                constexpr auto value = Meta::value_or_v<flag_list>;
                mcu_tca()->ctrlb.template add<value, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                AVR::PinGroup<pin_list>::template dir<AVR::Output>();
            }
            
            template<typename WO>
            static inline void onCompareMatch(auto f) {
                static_assert(WO::value <= 2);
                if constexpr(WO::value == 0) {
                    mcu_tca()->intflags.template testAndReset<mcu_timer_t::Intflags_t::cmp0>(f);
                }
                else if constexpr(WO::value == 1) {
                    mcu_tca()->intflags.template testAndReset<mcu_timer_t::Intflags_t::cmp1>(f);
                }
                else if constexpr(WO::value == 2) {
                    mcu_tca()->intflags.template testAndReset<mcu_timer_t::Intflags_t::cmp2>(f);
                }
            }
            
            static inline void ppmRaw(index_type ch, uint16_t v) {
                duty(ch, v);
            }
            
            template<typename T, auto L, auto U>
            static inline void ppm(index_type ch, etl::uint_ranged_NaN<T, L, U> raw) {
                if (raw) {
                    T v1 = raw.toInt() - L;
                    constexpr uint64_t denom = U - L;
                    constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
                    if constexpr(nom < denom) {
                        uint16_t ocr = etl::Rational::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
                        uint16_t v = clamp(ocr, ocMin, ocMax);
                        duty(ch, v);
                    }
                    else {
                        static_assert( (((10 * nom) / denom) * 255) <= std::numeric_limits<uint16_t>::max());
                        uint16_t ocr = ((v1 * ((10 * nom) / denom)) / 10) + ranged_type::Lower;
                        uint16_t v = clamp(ocr, ocMin, ocMax);
                        duty(ch, v);
                    }
                }
            }
            template<typename T, auto L, auto U>
            static inline void ppm(index_type ch, etl::uint_ranged<T, L, U> raw) {
                T v1 = raw.toInt() - L;
                constexpr uint64_t denom = U - L;
                constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
                if constexpr(nom < denom) {
                    uint16_t ocr = etl::Rational::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
                    uint16_t v = clamp(ocr, ocMin, ocMax);
                    duty(ch, v);
                }
                else {
                    static_assert( (((10 * nom) / denom) * 255) <= std::numeric_limits<uint16_t>::max());
                    uint16_t ocr = ((v1 * ((10 * nom) / denom)) / 10) + ranged_type::Lower;
                    uint16_t v = clamp(ocr, ocMin, ocMax);
                    duty(ch, v);
                }
            }
        private:
            inline static constexpr void duty(const index_type ch, const uint16_t d) {
                if (ch == 0) {
                    *mcu_tca()->cmp0buf = d;
                }
                else if (ch == 1) {
                    *mcu_tca()->cmp1buf = d;
                }
                else if (ch == 2) {
                    *mcu_tca()->cmp2buf = d;
                }
            }
        };
        
        template<auto TN, typename Pos, typename List, AVR::Concepts::At01Series MCU>
        struct PpmOut<AVR::Portmux::Position<AVR::Component::Tcb<TN>, Pos>, List, MCU> {
            using mcu_timer_t = typename MCU::TCB; 
            static constexpr auto mcu_tcb = AVR::getBaseAddr<mcu_timer_t, TN>;
            using position = AVR::Portmux::Position<AVR::Component::Tcb<TN>, Pos>;

            using pin = AVR::Portmux::Map<position>::wopin;

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
        
            static constexpr uint16_t ccPulse = 500_us * timerFrequency;
            static_assert(ccPulse > 0, "wrong oc value");
            static constexpr uint16_t ccPulseFrame = ocMax + ccPulse;
            static_assert(ccPulse > 0, "wrong oc value");

            using index_type = void;
            
            using ranged_type = etl::uint_ranged<uint16_t, ocMin, ocMax> ;
            
            template<bool start = true>
            static inline void init() {
                *mcu_tcb()->cnt = 0xffff;
                mcu_tcb()->ctrla.template set<mcu_timer_t::CtrlA_t::clktca | mcu_timer_t::CtrlA_t::enable>();
                mcu_tcb()->ctrlb.template set<mcu_timer_t::CtrlB_t::mode_single>(); // todo: OR Flags
                mcu_tcb()->ctrlb.template add<mcu_timer_t::CtrlB_t::ccmpen>();
                mcu_tcb()->evctrl.template set<mcu_timer_t::EvCtrl_t::captei>();
                pin::template dir<AVR::Output>();
                duty(ocMedium);
            }

            template<typename T, auto L, auto U>
            static bool ppm(etl::uint_ranged_NaN<T, L, U> raw) {
                if (raw) {
                    T v1 = raw.toInt() - L;
                    constexpr uint64_t denom = U - L;
                    constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
                    if constexpr(nom < denom) {
                        uint16_t ocr = etl::Rational::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
                        uint16_t v = clamp(ocr, ocMin, ocMax);
                        return duty(v);
                    }
                    else {
                        static_assert( (((10 * nom) / denom) * 255) <= std::numeric_limits<uint16_t>::max());
                        uint16_t ocr = ((v1 * ((10 * nom) / denom)) / 10) + ranged_type::Lower;
                        uint16_t v = clamp(ocr, ocMin, ocMax);
                        return duty(v);
                    }
                }
                return false;
            }
            
            static inline uint16_t da{};
            
            template<typename T, auto L, auto U>
            static void ppm_async(etl::uint_ranged_NaN<T, L, U> raw) {
                if (raw) {
                    T v1 = raw.toInt() - L;
                    constexpr uint64_t denom = U - L;
                    constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
                    if constexpr(nom < denom) {
                        uint16_t ocr = etl::Rational::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
                        da = clamp(ocr, ocMin, ocMax);
                    }
                    else {
                        static_assert( (((10 * nom) / denom) * 255) <= std::numeric_limits<uint16_t>::max());
                        uint16_t ocr = ((v1 * ((10 * nom) / denom)) / 10) + ranged_type::Lower;
                        da = clamp(ocr, ocMin, ocMax);
                    }
                }
            }
            static void ppmRaw(const uint16_t v) {
                da = v;
            }
            
            inline static void onReload(auto f) {
                if (!mcu_tcb()->status.template isSet<mcu_timer_t::Status_t::run>()) {
                    *mcu_tcb()->ccmp = da;
                    *mcu_tcb()->cnt = 0xffff;
                    f();
                }
            }
        private:
            inline static constexpr bool duty(const uint16_t d) {
                if (!mcu_tcb()->status.template isSet<mcu_timer_t::Status_t::run>()) {
                    *mcu_tcb()->ccmp = d;
                    return true;
                }
                return false;
            }   
        };
    }
}
