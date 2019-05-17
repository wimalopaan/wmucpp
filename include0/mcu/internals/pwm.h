#pragma once

#include <std/chrono>
#include <etl/rational.h>

#include "../common/isr.h"
#include "../common/pwm.h"
#include "../common/concepts.h"

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;
    
    namespace PWM {
        template<bool B>
        struct ChannelA : etl::NamedFlag<B>{};
        template<bool B>
        struct ChannelB : etl::NamedFlag<B>{};
        template<bool B>
        struct Overflow : etl::NamedFlag<B>{};
        
        template<etl::Concepts::NamedConstant TimerNumber, typename MCU = DefaultMcuType>
        struct StaticPwm;
        
        // WGM Mode 3
        // static pwm frequency (changing frequency gives glitches)
        // no ISR
        template<etl::Concepts::NamedConstant TimerNumber, typename MCU>
        requires ((TimerNumber::value == 0) || (TimerNumber::value == 2))
        struct StaticPwm<TimerNumber, MCU> final {
            using mcu_timer_type = mcu_timer_t<TimerNumber::value>;
            using value_type  = mcu_timer_value_t<TimerNumber::value>;        
            using ta = typename TimerParameter<TimerNumber::value, MCU>::ta;        
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;        
            
            using pwmA = typename TimerParameter<TimerNumber::value, MCU>::ocAPin;
            using pwmB = typename TimerParameter<TimerNumber::value, MCU>::ocBPin;
            
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
            
            inline static constexpr void a(const value_type& v) {
                *mcu_timer()->ocra = v;
            }
            inline static constexpr void b(const value_type& v) {
                *mcu_timer()->ocrb = v;
            }
   
            template<const hertz& Frequency>
            inline static constexpr void init() {
//                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber>(Frequency);
                constexpr uint16_t p = AVR::PWM::Util::prescalerForAbove<TimerNumber::value>(Frequency);
                
//                using x1 = std::integral_constant<uint16_t, p>;
//                x1::_;
                
                constexpr auto bits = bitsFrom<p>(prescaler_bits_v<TimerNumber::value>);            
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                // mode 3
                mcu_timer()->tccra.template set<ta::wgm0 | ta::wgm1 | ta::coma1 | ta::coma0 | ta::comb1 | ta::comb0>();
//                mcu_timer()->tccrb.template add<tb::wgm2, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                
//                mcu_timer_interrupts()->timsk.template add<mask_type::toie1 | mask_type::ocie1a | mask_type::ocie1b>();
                
                pwmA::template dir<AVR::Output>();
                pwmB::template dir<AVR::Output>();
            }
            
        };
        
        template<etl::Concepts::NamedConstant TimerNumber, typename MCU = DefaultMcuType>
        struct DynamicPwm;
        
        // WGM Mode 15
        // dynamic changeable frequnecy
        template<etl::Concepts::NamedConstant TimerNumber, typename MCU>
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
                ocAPin::template dir<AVR::Output>();
                ocBPin::template dir<AVR::Output>();
                ocAPin::off();
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
                *mcu_timer()->ocra = 0;
            }
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
            inline static constexpr void init() {
                constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber::value>(Frequency);
                static_assert(tsd, "wrong prescaler");
                
//                using x1 = std::integral_constant<uint16_t, tsd.prescaler>::_;;
//                using x2 = std::integral_constant<uint16_t, tsd.ocr>::_;
                
                constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<TimerNumber::value>);   
                
//                constexpr auto bits = tb::cs1;
                
                std::integral_constant<uint8_t, (uint8_t)bits>::_;
                
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                // mode 3
                mcu_timer()->tccra.template set<ta::wgm1 | ta::wgm0>();

                on();                
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
