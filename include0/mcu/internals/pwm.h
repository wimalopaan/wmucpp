#pragma once

#include <std/chrono>
#include <etl/types.h>
#include <etl/rational.h>
#include <etl/type_traits.h>

#include "../common/isr.h"
#include "../common/pwm.h"
#include "../common/concepts.h"

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
        
        template<typename P, AVR::Concepts::At01Series MCU>
        struct DynamicPwm<Portmux::Position<Component::Tca<0>, P>, MCU> final {
            
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
            
//            template<typename... Outs>
//            inline static constexpr void on() {
//                using out_list = Meta::transform_type<womapper, Meta::List<Outs...>>;
//                constexpr auto value = Meta::value_or_v<out_list>;
////                                std::integral_constant<decltype(value), value>::_;
//                mcu_tca()->ctrlb.template add<value>();
//            }
            template<Meta::concepts::List OutList, typename IMode = etl::RestoreState>
            inline static constexpr void on() {
                using out_list = Meta::transform_type<womapper, OutList>;
                constexpr auto value = Meta::value_or_v<out_list>;
//                                std::integral_constant<decltype(value), value>::_;
                mcu_tca()->ctrlb.template add<value, etl::DisbaleInterrupt<IMode>>();
            }

//            template<typename... Outs>
//            inline static constexpr void off() {
//                using out_list = Meta::transform_type<womapper, Meta::List<Outs...>>;
//                constexpr auto value = Meta::value_or_v<out_list>;
////                                std::integral_constant<decltype(value), value>::_;
//                mcu_tca()->ctrlb.template clear<value>();
//            }
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
            template<typename... Outs>
            inline static constexpr void duty(uint16_t d) {
                using out_list = Meta::List<Outs...>;
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
