#pragma once

#include <std/chrono>
#include <etl/rational.h>

#include "groups.h"

#include "../common/pwm.h"

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;
    
    template<auto TimerNumber, typename DB = void, typename MCU = DefaultMcuType>
    struct Pwm final {
        Pwm() = delete;
        using mcu_timer_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_type;
        using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;        
        using ta = typename TimerParameter<TimerNumber, MCU>::ta;        
        using tb = typename TimerParameter<TimerNumber, MCU>::tb;        
        
        using flags_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_interrupts_flags_type;
        
        using ocAPin = typename TimerParameter<TimerNumber, MCU>::ocAPin;
        using ocBPin = typename TimerParameter<TimerNumber, MCU>::ocBPin;
        
        static constexpr auto mcu_timer = TimerParameter<TimerNumber, MCU>::mcu_timer;
        static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber, MCU>::mcu_timer_interrupts;
        
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
            
            constexpr auto tsd = AVR::PWM::Util::calculate<TimerNumber>(Frequency);
            static_assert(tsd, "wrong prescaler");
            
            constexpr auto bits = bitsFrom<tsd.prescaler>(mcu_timer_type::template PrescalerBits<TimerNumber>::values);            
            static_assert(isset(bits), "wrong prescaler");
            mcu_timer()->tccrb.template set<bits>();

            // mode 15
            mcu_timer()->tccra.template set<ta::wgm0 | ta::wgm1 | ta::comb1>();
            mcu_timer()->tccrb.template add<tb::wgm2 | tb::wgm3, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            
            *mcu_timer()->ocrb = 0;
            *mcu_timer()->ocra = 0;
        }
        
        
//        inline static constexpr bool frequency(const hertz& f) {
//            auto tsd = AVR::PWM::Util::calculate<TimerNumber>(f);
//            if (!tsd) {
//                return false;
//            }
//            auto bits = bitsFrom(tsd.prescaler, mcu_timer_type::template PrescalerBits<TimerNumber>::values);
            
//            if (!isset(bits)) {
//                return false;
//            }
//            mcu_timer()->tccrb.template setPartial<tb::cs0 | tb::cs1 | tb::cs2>(bits);
//            *mcu_timer()->icr = tsd.ocr;
//            return true;
//        }
        
        
    private:
    };
}
