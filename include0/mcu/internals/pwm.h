#pragma once

#include <std/chrono>
#include <etl/types.h>
#include <etl/rational.h>
#include <etl/type_traits.h>

#include "../common/isr.h"
#include "../common/pwm.h"
#include "../common/concepts.h"

#include "ccp.h"
#include "timer.h"

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;
    
    namespace PWM {
        template<uint8_t N> struct Resolution;
        template<> struct Resolution<8> {};
        template<> struct Resolution<9> {};
        template<> struct Resolution<10> {};
        template<> struct Resolution<16> {};
        
        
        template<bool B>
        struct ChannelA : etl::NamedFlag<B>{};
        template<bool B>
        struct ChannelB : etl::NamedFlag<B>{};
        template<bool B>
        struct Overflow : etl::NamedFlag<B>{};
        
        template<etl::Concepts::NamedConstant TimerNumber, typename ListOfChannels = Meta::List<AVR::A, AVR::B>, typename Reso = Resolution<8>, typename MCU = DefaultMcuType>
        struct StaticPwm;
        
        // mode14
        template<etl::Concepts::NamedConstant TimerNumber, typename... Channels, AVR::Concepts::AtMega_X MCU>
        requires ((TimerNumber::value == 1) || (TimerNumber::value == 3) || (TimerNumber::value == 4))
        struct StaticPwm<TimerNumber, Meta::List<Channels...>, Resolution<16>, MCU> final {
            using mcu_timer_type = mcu_timer_t<TimerNumber::value>;
            
            using ta = typename TimerParameter<TimerNumber::value, MCU>::ta;        
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;        
            
            inline static constexpr tb csBitMask = {tb::cs2 | tb::cs1 | tb::cs0};

            using pwmA = typename TimerParameter<TimerNumber::value, MCU>::ocAPin;
            using pwmB = typename TimerParameter<TimerNumber::value, MCU>::ocBPin;
            
            using channel_list = Meta::List<Channels...>;
            static_assert(Meta::is_set<channel_list>::value);
            static_assert(Meta::size_v<channel_list> > 0);
            static_assert(Meta::size_v<channel_list> <= 2);
            
            using flags_type = mcu_timer_interrupts_flags_t<TimerNumber::value>;
            
            static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;

            template<const hertz& Frequency>
            struct Fixed {
                static inline constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                
                using base_type  = mcu_timer_value_t<TimerNumber::value>;        
                
                using value_type = etl::uint_ranged<base_type, 0, tsd.ocr>;
                
                static inline constexpr value_type max() {
                    return tsd.ocr;
                }
                
                template<bool visible = Meta::contains<channel_list, AVR::A>::value>
                inline static constexpr 
                        std::enable_if_t<visible>
                        a(const value_type& v) {
                    *mcu_timer()->ocra = v;
                }
                template<bool visible = Meta::contains<channel_list, AVR::B>::value>
                inline static constexpr 
                        std::enable_if_t<visible>
                        b(const value_type& v) {
                    *mcu_timer()->ocrb = v;
                }
                
                inline static hertz frequency() {
                    return Frequency;
                }
                
                inline static constexpr void init() {
//                    std::integral_constant<uint16_t, tsd.prescaler>::_;
//                    std::integral_constant<uint16_t, tsd.ocr>::_;
                    
                    constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<TimerNumber::value>);            
                    static_assert(isset(bits), "wrong prescaler");
                    mcu_timer()->tccrb.template set<bits>();
                    *mcu_timer()->icr = tsd.ocr;
                    
                    // mode 14
                    mcu_timer()->tccra.template set<ta::wgm1 | ta::coma1 | ta::comb1>();
                    mcu_timer()->tccrb.template add<tb::wgm2 | tb::wgm3>();
                    
                    if constexpr(Meta::contains<channel_list, AVR::A>::value) {
                        pwmA::template dir<AVR::Output>();
                    }
                    if constexpr(Meta::contains<channel_list, AVR::B>::value) {
                        pwmB::template dir<AVR::Output>();
                    }
                }
            };
            
       };
        
        // WGM Mode 3/5 (bei 8-Bit)
        // static pwm frequency (changing frequency gives glitches)
        // no ISR
        template<etl::Concepts::NamedConstant TimerNumber, typename... Channels, AVR::Concepts::AtMega_X MCU>
        struct StaticPwm<TimerNumber, Meta::List<Channels...>, Resolution<8>, MCU> final {
            using mcu_timer_type = mcu_timer_t<TimerNumber::value>;
            //            using value_type  = mcu_timer_value_t<TimerNumber::value>;        
            using value_type  = uint8_t;        
            using ta = typename TimerParameter<TimerNumber::value, MCU>::ta;        
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;        
            
            inline static constexpr tb csBitMask = {tb::cs2 | tb::cs1 | tb::cs0};
            
            using pwmA = typename TimerParameter<TimerNumber::value, MCU>::ocAPin;
            using pwmB = typename TimerParameter<TimerNumber::value, MCU>::ocBPin;
            
            using channel_list = Meta::List<Channels...>;
            static_assert(Meta::is_set<channel_list>::value);
            static_assert(Meta::size_v<channel_list> > 0);
            static_assert(Meta::size_v<channel_list> <= 2);
            
            using flags_type = mcu_timer_interrupts_flags_t<TimerNumber::value>;
            //            using mask_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_mask_type;
            
            static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;
            
            template<AVR::Concepts::McuPart T>
            struct Channel {
                using pwm = StaticPwm<TimerNumber, MCU>;
                using value_type = typename StaticPwm<TimerNumber, MCU>::value_type;
                
                inline static constexpr void set(const value_type& v) {
                    if constexpr(std::is_same_v<T, AVR::A>) {
                        pwm::a(v);
                    }
                    else if constexpr(std::is_same_v<T, AVR::B>) {
                        pwm::b(v);
                    }
                    else {
                        static_assert(std::false_t<T>::value, "wrong channel");
                    }
                }
            };
            
            template<bool visible = Meta::contains<channel_list, AVR::A>::value>
            inline static constexpr 
            std::enable_if_t<visible>
            a(const value_type& v) {
                *mcu_timer()->ocra = v;
            }
            template<bool visible = Meta::contains<channel_list, AVR::B>::value>
            inline static constexpr 
            std::enable_if_t<visible>
            b(const value_type& v) {
                *mcu_timer()->ocrb = v;
            }
            
            inline static hertz frequency() {
                return Project::Config::fMcu / (uint32_t)prescaler();
            }
            
            inline static auto prescaler() {
                const auto bits = mcu_timer()->tccrb.template get<csBitMask>();
                return AVR::PWM::Util::bitsToPrescale(bits, AVR::Util::Timer::prescaler_bits_v<TimerNumber::value>);
            }
            
            template<const hertz& Frequency>
            inline static constexpr void init() {
                //                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber>(Frequency);
                constexpr uint16_t p = AVR::PWM::Util::prescalerForAbove<TimerNumber::value>(Frequency);
                
                //                                using x1 = std::integral_constant<uint16_t, p>;
                //                                x1::_;
                
                constexpr auto bits = bitsFrom<p>(prescaler_bits_v<TimerNumber::value>);            
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                if constexpr(TimerNumber::value == 3) {
                    // mode 5
                    mcu_timer()->tccra.template set<ta::wgm0 | ta::coma1 | ta::comb1>();
                    mcu_timer()->tccrb.template add<tb::wgm2 >();
                }
                else {
                    // mode 3
                    mcu_timer()->tccra.template set<ta::wgm0 | ta::wgm1 | ta::coma1 | ta::coma0 | ta::comb1 | ta::comb0>();
                }
                
                if constexpr(Meta::contains<channel_list, AVR::A>::value) {
                    pwmA::template dir<AVR::Output>();
                }
                if constexpr(Meta::contains<channel_list, AVR::B>::value) {
                    pwmB::template dir<AVR::Output>();
                }
            }
            
        };
        
        template<typename TimerNumber, typename MCU = DefaultMcuType>
        struct DynamicPwm;
        template<typename TimerNumber, typename MCU = DefaultMcuType>
        struct DynamicPwm8Bit;
        
        // WGM Mode 15
        // dynamic changeable frequnecy
        template<etl::Concepts::NamedConstant TimerNumber, AVR::Concepts::AtMega MCU>
        requires ((TimerNumber::value == 1) || (TimerNumber::value == 3) || (TimerNumber::value == 4))
        struct DynamicPwm<TimerNumber, MCU> final {
            DynamicPwm() = delete;
            using mcu_timer_type = mcu_timer_t<TimerNumber::value>;
            using value_type  = mcu_timer_value_t<TimerNumber::value>;        
            using ta = typename TimerParameter<TimerNumber::value, MCU>::ta;        
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;        
            
            using flags_type = mcu_timer_interrupts_flags_t<TimerNumber::value>;
            
            using ocAPin = typename TimerParameter<TimerNumber::value, MCU>::ocAPin;
            using ocBPin = typename TimerParameter<TimerNumber::value, MCU>::ocBPin;
            
            static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;
            
            inline static constexpr void period(value_type v) {
                *mcu_timer()->ocra = v;
            }
            
            inline static constexpr void on(value_type v) {
                *mcu_timer()->ocrb = v;
            }
            
            inline static constexpr void reverse(bool r) {
                if (r) {
                    mcu_timer()->tccra.template add<ta::comb0>();
                    ocAPin::on();
                }
                else {
                    mcu_timer()->tccra.template clear<ta::comb0>();
                    ocAPin::off();
                }
            }
            
            inline static constexpr void duty(value_type v, value_type p) {
                *mcu_timer()->ocra = p;
                *mcu_timer()->ocrb = v;
            }
            
            template<const hertz& Frequency>
            inline static constexpr void init() {
                //                ocAPin::template dir<AVR::Output>();
                ocBPin::template dir<AVR::Output>();
                //                ocAPin::off();
                ocBPin::off();
                
                constexpr auto tsd = AVR::Util::Timer::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                
                constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<TimerNumber::value>);            
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                // mode 15
                mcu_timer()->tccra.template set<ta::wgm0 | ta::wgm1 | ta::comb1>();
                mcu_timer()->tccrb.template add<tb::wgm2 | tb::wgm3, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                
                *mcu_timer()->ocrb = 0;
                *mcu_timer()->ocra = tsd.ocr;
            }
        };
        
        
        template<auto N>
        struct WO final : etl::NamedConstant<N> {};

        template<typename P, AVR::Concepts::AtTiny1 MCU>
        struct DynamicPwm<Portmux::Position<Component::Tcd<0>, P>, MCU> final {
            DynamicPwm() = delete;

            using value_type = etl::uint_ranged<uint16_t, 0, 4095>;
            
            inline static constexpr auto f_timer = Project::Config::fMcu;

            using mcu_timer_t = typename MCU::TCD; 
            static constexpr auto mcu_tcd = getBaseAddr<mcu_timer_t, 0>;
            
            using position = Portmux::Position<Component::Tcd<0>, P>;
            
            using CtrlA2_t = mcu_timer_t::CtrlA2_t;
            using CtrlA4_t = mcu_timer_t::CtrlA4_t;
            using CtrlB_t = mcu_timer_t::CtrlB_t;
            using CtrlE_t = mcu_timer_t::CtrlE_t;
            using Status_t = mcu_timer_t::Status_t;
            using FaultCtrl_t = mcu_timer_t::FaultCtrl_t;
            
            using pins = Meta::List<typename AVR::Portmux::Map<position>::wo0pin, typename AVR::Portmux::Map<position>::wo1pin>;
            
//            pins::_;
            
            using ccp = AVR::Cpu::Ccp<MCU>;

            static inline auto status_r = []()->auto&{return mcu_tcd()->status;};
            static inline auto ctrla_r = []()->auto&{return mcu_tcd()->ctrla;};
            static inline auto ctrlb_r = []()->auto&{return mcu_tcd()->ctrlb;};
            static inline auto ctrle_r = []()->auto&{return mcu_tcd()->ctrle;};
            static inline auto faultctrl_r = []()->auto&{return mcu_tcd()->faultctrl;};
            
            inline static constexpr void init() {
                AVR::set<CtrlA2_t::div32>(ctrla_r());
                AVR::set<CtrlB_t::oneRamp>(ctrlb_r());
                ccp::unlock([]{
                    AVR::set<FaultCtrl_t::cmpaen | FaultCtrl_t::cmpben>(faultctrl_r());
                });
                waitFor<Status_t::enready>(status_r());
                AVR::add<CtrlA4_t::enable>(ctrla_r());
                AVR::PinGroup<pins>::template dir<Output>();
            }
            inline static constexpr void frequency(const External::Units::hertz& f) {
                waitFor<Status_t::cmdready>(status_r());
                *mcu_tcd()->cmpaset = 0;
                *mcu_tcd()->cmpbclr = Config::fMcu / (f * 32);
                AVR::set<CtrlE_t::synceoc>(ctrle_r());
                mMax.set(Config::fMcu / (32 * f));
            }
//            inline static constexpr void frequency(const uint16_t& f) {
//                *mcu_tcd()->cmpseta = 0;
//                *mcu_tcd()->cmpclra = f;
//            }
            template<typename... Outs>
            inline static constexpr void duty(value_type d) {
                waitFor<Status_t::cmdready>(status_r());
                *mcu_tcd()->cmpaclr = d;
                *mcu_tcd()->cmpbset = mMax - d;
                AVR::set<CtrlE_t::synceoc>(ctrle_r());
            }
            template<etl::Concepts::NamedConstant Out>
            inline static constexpr void set(value_type v) {
                waitFor<Status_t::cmdready>(status_r());
                if constexpr(Out::value == 0) {
                    *mcu_tcd()->cmpaset = v;
                }                
                else if constexpr(Out::value == 1) {
                    *mcu_tcd()->cmpbset = v;
                }                
                else {
                    static_assert(std::false_v<MCU>);
                }
                AVR::set<CtrlE_t::synceoc>(ctrle_r());
            }
            template<etl::Concepts::NamedConstant Out>
            inline static constexpr void clear(value_type v) {
                waitFor<Status_t::cmdready>(status_r());
                if constexpr(Out::value == 0) {
                    *mcu_tcd()->cmpaclr = v;
                }                
                else {
                    static_assert(std::false_v<MCU>);
                }
                AVR::set<CtrlE_t::synceoc>(ctrle_r());
            }
            template<Meta::concepts::List OutList, typename IMode = etl::RestoreState>
            inline static constexpr void on() {
                if constexpr(Meta::contains_v<OutList, WO<0>>) {
                    ccp::unlock([]{
                        AVR::set<FaultCtrl_t::cmpaen>(faultctrl_r());
                    });                    
                }
                if constexpr(Meta::contains_v<OutList, WO<1>>) {
                    ccp::unlock([]{
                        AVR::set<FaultCtrl_t::cmpben>(faultctrl_r());
                    });                                        
                }
            }
            template<Meta::concepts::List OutList, typename IMode = etl::RestoreState>
            inline static constexpr void off() {
                if constexpr(Meta::contains_v<OutList, WO<0>>) {
                    ccp::unlock([]{
                        AVR::clear<FaultCtrl_t::cmpaen>(faultctrl_r());
                    });                    
                }
                if constexpr(Meta::contains_v<OutList, WO<1>>) {
                    ccp::unlock([]{
                        AVR::clear<FaultCtrl_t::cmpben>(faultctrl_r());
                    });                                        
                }
            }
            inline static constexpr value_type max() {
                return mMax;
            }
        private:
            inline static value_type mMax{};
        };

        namespace helper {
            struct Normal;
            struct Split;
            template<typename Position, uint8_t N, typename Mode = Normal, typename MCU = DefaultMcuType> struct WOMapper;
            template<typename Position, typename MCU>
            struct WOMapper<Position, 0, Normal, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo0pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::cmp0en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position, typename MCU>
            struct WOMapper<Position, 1, Normal, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo1pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::cmp1en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position, typename MCU>
            struct WOMapper<Position, 2, Normal, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo2pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::cmp2en;
                using type = std::integral_constant<decltype(value), value>;
            };

            template<typename Position, typename MCU>
            struct WOMapper<Position, 0, Split, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo0pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::scmp0en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position, typename MCU>
            struct WOMapper<Position, 1, Split, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo1pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::scmp1en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position, typename MCU>
            struct WOMapper<Position, 2, Split, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo2pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::scmp2en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position, typename MCU>
            struct WOMapper<Position, 3, Split, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo3pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::scmp3en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position, typename MCU>
            struct WOMapper<Position, 4, Split, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo4pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::scmp4en;
                using type = std::integral_constant<decltype(value), value>;
            };
            template<typename Position, typename MCU>
            struct WOMapper<Position, 5, Split, MCU> {
                using pin = typename AVR::Portmux::Map<Position>::wo5pin;
                inline static constexpr auto value = MCU::TCA::CtrlB_t::scmp5en;
                using type = std::integral_constant<decltype(value), value>;
            };
            
            template<typename WOM>
            struct pinToType {
                using type = typename WOM::pin;
            };
            
        }
        
        template<typename P, AVR::Concepts::At01Series MCU>
        struct DynamicPwm8Bit<Portmux::Position<Component::Tca<0>, P>, MCU> final {
            DynamicPwm8Bit() = delete;
            
            using position = Portmux::Position<Component::Tca<0>, P>;
            
            using b_channels = Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>;
            using a_channels = Meta::List<AVR::PWM::WO<3>, AVR::PWM::WO<4>, AVR::PWM::WO<5>>;
            using all_channels = Meta::concat<a_channels, b_channels>;
            
            inline static constexpr auto all_mask = []<typename... W>(Meta::List<W...>){
                return (helper::WOMapper<position, W::value, helper::Split>::value | ...);
            }(all_channels{});
            
//            std::integral_constant<decltype(all_mask), all_mask>::_;
            
            using value_type = uint8_t;
            
            using mcu_timer_t = typename MCU::TCA; 
            static constexpr auto mcu_tca = getBaseAddr<mcu_timer_t, 0>;

            template<typename WO>
            using getPin = helper::pinToType<helper::WOMapper<position, WO::value, helper::Split>>;
            
            using a_pins = Meta::transform_type<getPin, a_channels>;
            using b_pins = Meta::transform_type<getPin, b_channels>;
            
//            pins::_;
            
            inline static constexpr auto pv = []{
                return MCU::TCA::prescalerValues[3].bits;
            }();
            
            using index_type = etl::uint_ranged<uint8_t, 0, 5>;
            
            inline static void on(const index_type c) {
                if (c == 0) {
                    on<0>();
                }
                else if (c == 1) {
                    on<1>();
                }
                else if (c == 2) {
                    on<2>();
                }
                else if (c == 3) {
                    on<3>();
                }
                else if (c == 4) {
                    on<4>();
                }
                else if (c == 5) {
                    on<5>();
                }
            }
            inline static void off(const index_type c) {
                if (c == 0) {
                    off<0>();
                }
                else if (c == 1) {
                    off<1>();
                }
                else if (c == 2) {
                    off<2>();
                }
                else if (c == 3) {
                    off<3>();
                }
                else if (c == 4) {
                    off<4>();
                }
                else if (c == 5) {
                    off<5>();
                }
            }
            
            template<auto N>
            inline static void on() {
                if constexpr(N == 0) {
                    mcu_tca()->ctrlb.template add<MCU::TCA::CtrlB_t::scmp0en>();
                }                
                if constexpr(N == 1) {
                    mcu_tca()->ctrlb.template add<MCU::TCA::CtrlB_t::scmp1en>();
                }                
                if constexpr(N == 2) {
                    mcu_tca()->ctrlb.template add<MCU::TCA::CtrlB_t::scmp2en>();
                }                
                if constexpr(N == 3) {
                    mcu_tca()->ctrlb.template add<MCU::TCA::CtrlB_t::scmp3en>();
                }                
                if constexpr(N == 4) {
                    mcu_tca()->ctrlb.template add<MCU::TCA::CtrlB_t::scmp4en>();
                }                
                if constexpr(N == 5) {
                    mcu_tca()->ctrlb.template add<MCU::TCA::CtrlB_t::scmp5en>();
                }                
            }
            template<auto N>
            inline static void off() {
                if constexpr(N == 0) {
                    mcu_tca()->ctrlb.template clear<MCU::TCA::CtrlB_t::scmp0en>();
                }                
                if constexpr(N == 1) {
                    mcu_tca()->ctrlb.template clear<MCU::TCA::CtrlB_t::scmp1en>();
                }                
                if constexpr(N == 2) {
                    mcu_tca()->ctrlb.template clear<MCU::TCA::CtrlB_t::scmp2en>();
                }                
                if constexpr(N == 3) {
                    mcu_tca()->ctrlb.template clear<MCU::TCA::CtrlB_t::scmp3en>();
                }                
                if constexpr(N == 4) {
                    mcu_tca()->ctrlb.template clear<MCU::TCA::CtrlB_t::scmp4en>();
                }                
                if constexpr(N == 5) {
                    mcu_tca()->ctrlb.template clear<MCU::TCA::CtrlB_t::scmp5en>();
                }                
            }
            
            inline static constexpr value_type pwmMax = 99;
            
            inline static void pwm(const index_type c, const uint8_t v) {
                if (c == 0) {
                    pwm<0>(v);
                }
                else if (c == 1) {
                    pwm<1>(v);
                }
                else if (c == 2) {
                    pwm<2>(v);
                }
                else if (c == 3) {
                    pwm<3>(v);
                }
                else if (c == 4) {
                    pwm<4>(v);
                }
                else if (c == 5) {
                    pwm<5>(v);
                }
            }
            
            template<auto N>
            inline static void pwm(const uint8_t vv) {
                auto v = std::clamp(vv, value_type{0}, pwmMax);
                if constexpr(N == 0) {
                    lset(*mcu_tca()->cmp0, v);
                }
                else if constexpr(N == 1) {
                    lset(*mcu_tca()->cmp1, v);
                }
                else if constexpr(N == 2) {
                    lset(*mcu_tca()->cmp2, v);
                }
                else if constexpr(N == 3) {
                    hset(*mcu_tca()->cmp0, v);
                }
                else if constexpr(N == 4) {
                    hset(*mcu_tca()->cmp1, v);
                }
                else if constexpr(N == 5) {
                    hset(*mcu_tca()->cmp2, v);
                }
            }
            
            inline static constexpr void init() {
                AVR::PinGroup<a_pins>::template dir<Output>();
                AVR::PinGroup<b_pins>::template dir<Output>();
                
                mcu_tca()->ctrld.template set<mcu_timer_t::CtrlD_t::splitm>();
                lhset(*mcu_tca()->per, pwmMax, pwmMax);
//                mcu_tca()->ctrlb.template set<all_mask>();
                mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA_t::enable | pv>();
            }
        private:
            template<typename T>
            inline static void lhset(T& r, const uint8_t low, const uint8_t high) {
                r = (high << 8) | low;
            }
            template<typename T>
            inline static void lset(T& r, const uint8_t low) {
                r = (r & 0xff00) | low;
            }
            template<typename T>
            inline static void hset(T& r, const uint8_t high) {
                r = (high << 8) | (r & 0x00ff);
            }
        };
        
        template<typename P, AVR::Concepts::At01Series MCU>
        struct DynamicPwm<Portmux::Position<Component::Tca<0>, P>, MCU> final {
            
            using all_channels = Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>;
            
            using value_type = uint16_t;
            
            inline static constexpr auto f_timer = Project::Config::fMcu;
            
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
            
            DynamicPwm() = delete;
            
            using mcu_timer_t = typename MCU::TCA; 
            static constexpr auto mcu_tca = getBaseAddr<mcu_timer_t, 0>;
            using position = Portmux::Position<Component::Tca<0>, P>;
            
            static inline constexpr auto cmpMask = mcu_timer_t::CtrlB_t::cmp2en | mcu_timer_t::CtrlB_t::cmp1en | mcu_timer_t::CtrlB_t::cmp0en;
            
            template<typename WO>
            using womapper = WOMapper<position, WO::value>;
            
            template<typename WO>
            struct pinmapper {
                using type = typename WOMapper<position, WO::value>::pin;
            };
            
            using pins = Meta::List<typename pinmapper<WO<0>>::type, typename pinmapper<WO<1>>::type, typename pinmapper<WO<2>>::type>;
            
            using f = Meta::front<pins>;
            
            inline static constexpr void init() {
                mcu_tca()->ctrla.template set<mcu_timer_t::CtrlA_t::enable>();
                //                mcu_tca()->ctrlb.template set<MCU::TCA::CtrlB_t::cmp2en | MCU::TCA::CtrlB_t::pwm>();
                mcu_tca()->ctrlb.template set<mcu_timer_t::CtrlB_t::pwm>();
                
                AVR::PinGroup<pins>::template dir<Output>();
            }
            
            template<Meta::concepts::List OutList, typename IMode = etl::RestoreState>
            inline static constexpr void on() {
                using out_list = Meta::transform_type<womapper, OutList>;
                constexpr auto value = Meta::value_or_v<out_list>;
//                                std::integral_constant<decltype(value), value>::_;
                mcu_tca()->ctrlb.template add<value, etl::DisbaleInterrupt<IMode>>();
            }

            template<Meta::concepts::List OutList, typename IMode = etl::RestoreState>
            inline static constexpr void off() {
                using out_list = Meta::transform_type<womapper, OutList>;
                constexpr auto value = Meta::value_or_v<out_list>;
//                                std::integral_constant<decltype(value), value>::_;
                mcu_tca()->ctrlb.template clear<value, etl::DisbaleInterrupt<IMode>>();
            }
            
            template<typename... Outs>
            inline static constexpr void set() {
                using pin_list = Meta::transform_type<pinmapper, Meta::List<Outs...>>;
                //                pin_list::_;
                off<Outs...>();
                AVR::PinGroup<pin_list>::on();
            }
            template<typename... Outs>
            inline static constexpr void reset() {
                using pin_list = Meta::transform_type<pinmapper, Meta::List<Outs...>>;
                //                pin_list::_;
                AVR::PinGroup<pin_list>::off();
            }
            
            inline static constexpr void frequency(const External::Units::hertz& f) {
                *mcu_tca()->perbuf = Config::fMcu / f;
                mMax = Config::fMcu / f;
            }
            inline static constexpr void frequency(const uint16_t& f) {
                *mcu_tca()->perbuf = f;
            }
            inline static constexpr value_type max() {
                return mMax;
            }
            template<Meta::concepts::List OutList>
            inline static constexpr void duty(uint16_t d) {
                using out_list = OutList;
                if constexpr(Meta::contains<out_list, WO<0>>::value) {
                    *mcu_tca()->cmp0buf = d;
                }
                if constexpr(Meta::contains<out_list, WO<1>>::value) {
                    *mcu_tca()->cmp1buf = d;
                }
                if constexpr(Meta::contains<out_list, WO<2>>::value) {
                    *mcu_tca()->cmp2buf = d;
                }
            }
        private:
            inline static value_type mMax = 0;
        };
        
        template<etl::Concepts::NamedConstant TimerNumber, 
                 etl::Concepts::NamedFlag UseA = etl::NamedFlag<true>, etl::Concepts::NamedFlag UseB = etl::NamedFlag<true>, 
                 typename MCU = DefaultMcuType>
        struct ISRPwm;
        
        // WGM Mode 3
        // static pwm frequency (changing frequency gives glitches)
        // must use ISR!
        
        template<etl::Concepts::NamedConstant TimerNumber, etl::Concepts::NamedFlag UseA, etl::Concepts::NamedFlag UseB, typename MCU>
        requires ((TimerNumber::value == 0))
        struct ISRPwm<TimerNumber, UseA, UseB, MCU> final {
            ISRPwm() = delete;
            using mcu_timer_type = mcu_timer_t<TimerNumber::value>;
            using value_type  = mcu_timer_value_t<TimerNumber::value>;        
            using ta = typename TimerParameter<TimerNumber::value, MCU>::ta;        
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;        
            
            using flags_type = mcu_timer_interrupts_flags_t<TimerNumber::value>;
            using mask_type = mcu_timer_interrupts_mask_t<TimerNumber::value>;
            
            static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;
            static constexpr uint8_t number = TimerNumber::value;
            
            template<bool visible = UseA::value>
            inline static constexpr 
                    std::enable_if_t<visible> a(const value_type& v) {
                *mcu_timer()->ocra = v;
            }
            template<bool visible = UseB::value>
            inline static constexpr
                    std::enable_if_t<visible> b(const value_type& v) {
                *mcu_timer()->ocrb = v;
            }
            
            template<const hertz& Frequency>
            inline static constexpr value_type max() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                return tsd.ocr;
            }
            
            template<const hertz& Frequency>
            inline static constexpr void f() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                
                //                using x1 = std::integral_constant<uint16_t, tsd.prescaler>::_;;
                //                using x2 = std::integral_constant<uint16_t, tsd.ocr>::_;
                
                constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<TimerNumber::value>);   
                
                //                std::integral_constant<uint8_t, (uint8_t)bits>::_;
                
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
            }
            
            template<const hertz& Frequency>
            inline static constexpr void init() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                
                //                using x1 = std::integral_constant<uint16_t, tsd.prescaler>::_;;
                //                using x2 = std::integral_constant<uint16_t, tsd.ocr>::_;
                
                constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<TimerNumber::value>);   
                
                //                std::integral_constant<uint8_t, (uint8_t)bits>::_;
                
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                // mode 3
                mcu_timer()->tccra.template set<ta::wgm1 | ta::wgm0>();
            }
            
            template<bool B, bool visible = UseA::value>
            static inline std::enable_if_t<visible> enable(ChannelA<B>) {
                if constexpr(B) {
                    mcu_timer_interrupts()->timsk.template add<mask_type::ociea>();
                }
                else {
                    mcu_timer_interrupts()->timsk.template clear<mask_type::ociea>();
                }
            }
            template<bool B, bool visible = UseB::value>
            static inline std::enable_if_t<visible> enable(ChannelB<B>) {
                if constexpr(B) {
                    mcu_timer_interrupts()->timsk.template add<mask_type::ocieb>();
                }
                else {
                    mcu_timer_interrupts()->timsk.template clear<mask_type::ocieb>();
                }
            }
            template<bool B>
            static inline void enable(Overflow<B>) {
                if constexpr(B) {
                    mcu_timer_interrupts()->timsk.template add<mask_type::toie>();
                }
                else {
                    mcu_timer_interrupts()->timsk.template clear<mask_type::toie>();
                }
            }
            
            inline static constexpr void off() {
                mcu_timer_interrupts()->timsk.template clear<mask_type::toie>();
                if constexpr(UseA::value) {
                    mcu_timer_interrupts()->timsk.template clear<mask_type::ociea>();
                }
                if constexpr(UseB::value) {
                    mcu_timer_interrupts()->timsk.template clear<mask_type::ocieb>();
                }
            } 
            
            inline static constexpr void on() {
                mcu_timer_interrupts()->timsk.template add<mask_type::toie>();
                if constexpr(UseA::value) {
                    mcu_timer_interrupts()->timsk.template add<mask_type::ociea>();
                }
                if constexpr(UseB::value) {
                    mcu_timer_interrupts()->timsk.template add<mask_type::ocieb>();
                }
            } 
        };
        
        template<etl::Concepts::NamedConstant TimerNumber, etl::Concepts::NamedFlag UseA, etl::Concepts::NamedFlag UseB, typename MCU>
        requires ((TimerNumber::value == 1) || (TimerNumber::value == 3))
        struct ISRPwm<TimerNumber, UseA, UseB, MCU> final {
            ISRPwm() = delete;
            using mcu_timer_type = mcu_timer_t<TimerNumber::value>;
            using value_type  = mcu_timer_value_t<TimerNumber::value>;        
            using ta = typename TimerParameter<TimerNumber::value, MCU>::ta;        
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;        
            
            using flags_type = mcu_timer_interrupts_flags_t<TimerNumber::value>;
            using mask_type = mcu_timer_interrupts_mask_t<TimerNumber::value>;
            
            static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;
            static constexpr uint8_t number = TimerNumber::value;
            
            inline static constexpr void a(const value_type& v) {
                *mcu_timer()->ocra = v;
            }
            inline static constexpr void b(const value_type& v) {
                *mcu_timer()->ocrb = v;
            }
            
            template<const hertz& Frequency>
            inline static constexpr value_type max() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                return tsd.ocr;
            }
            
            template<const hertz& Frequency>
            inline static constexpr void init() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                
                //            using x1 = std::integral_constant<uint16_t, tsd.prescaler>;
                //            using x2 = std::integral_constant<uint16_t, tsd.ocr>;
                //            x2::_;
                
                constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<TimerNumber::value>);            
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                *mcu_timer()->icr = tsd.ocr - 1;
                
                // mode 14
                mcu_timer()->tccra.template set<ta::wgm1>();
                mcu_timer()->tccrb.template add<tb::wgm2 | tb::wgm3, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                
                mcu_timer_interrupts()->timsk.template add<mask_type::toie | mask_type::ociea | mask_type::ocieb>();
            }
        };
        
        template<etl::Concepts::NamedConstant TimerNumber, etl::Concepts::NamedFlag UseA, etl::Concepts::NamedFlag UseB, AVR::Concepts::AtMega_8 MCU>
        requires (TimerNumber::value == 1)
        struct ISRPwm<TimerNumber, UseA, UseB, MCU> final {
            ISRPwm() = delete;
            using mcu_timer_type = mcu_timer_t<TimerNumber::value>;
            using value_type  = mcu_timer_value_t<TimerNumber::value>;        
            using ta = typename TimerParameter<TimerNumber::value, MCU>::ta;        
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;        
            
            using flags_type = mcu_timer_interrupts_flags_t<TimerNumber::value>;
            using mask_type = mcu_timer_interrupts_mask_t<TimerNumber::value>;
            
            static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;
            static constexpr uint8_t number = TimerNumber::value;
            
            inline static constexpr void a(const value_type& v) {
                *mcu_timer()->ocra = v;
            }
            inline static constexpr void b(const value_type& v) {
                *mcu_timer()->ocrb = v;
            }
            
            template<const hertz& Frequency>
            inline static constexpr value_type max() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                return tsd.ocr;
            }
            
            template<const hertz& Frequency>
            inline static constexpr void init() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                
                //            using x1 = std::integral_constant<uint16_t, tsd.prescaler>;
                //            using x2 = std::integral_constant<uint16_t, tsd.ocr>;
                //            x2::_;
                
                constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<TimerNumber::value>);            
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                *mcu_timer()->icr = tsd.ocr - 1;
                
                // mode 14
                mcu_timer()->tccra.template set<ta::wgm1>();
                mcu_timer()->tccrb.template add<tb::wgm2 | tb::wgm3, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                
                mcu_timer_interrupts()->timsk.template add<mask_type::toie1 | mask_type::ocie1a | mask_type::ocie1b>();
            }
        };
        
    }
}
