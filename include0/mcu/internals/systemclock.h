#pragma once

#include <std/chrono>

#include "groups.h"

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;
    
    template<int Number, const std::chrono::milliseconds& Interval, typename MCU = DefaultMcuType>
    struct SystemTimer {
        using mcu_timer_type = typename TimerParameter<Number, MCU>::mcu_timer_type;
        using tccra_type = typename TimerParameter<Number, MCU>::ta;        
        using tccrb_type = typename TimerParameter<Number, MCU>::tb;        
        using value_type  = typename TimerParameter<Number, MCU>::value_type;        
        using interrupts_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_type;
        using flags_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_flags_type;
        
        static constexpr auto mcuTimer = TimerParameter<Number, MCU>::mcuTimer;
        static constexpr auto mcuInterrupts = TimerParameter<Number, MCU>::mcuInterrupts;
        
        static constexpr auto intervall = Interval;
        static constexpr auto frequency = uint16_t{1} / Interval;
        static constexpr auto tsd = AVR::Util::Timer::calculate<mcu_timer_type, Number>(frequency);
        static_assert(tsd, "falscher wert für p");
        
        template<uint16_t PreScale>
        inline static void prescale() {
            constexpr tccrb_type bits = AVR::Util::Timer::bitsFrom<PreScale>(MCU::Timer8Bit::template PrescalerBits<Number>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcuTimer()->tccrb.template set<bits>();
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
        }
        
        inline static void init() {
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
            prescale<tsd.prescaler>();
            *mcuTimer()->ocra = tsd.ocr - 1;
        }

        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            if (mcuInterrupts()->tifr.template isSet<flags_type::ocfa>()) {
                f();
                mcuInterrupts()->tifr.template reset<flags_type::ocfa>(); // reset
            } 
        }
    };
    template<int Number, const std::chrono::milliseconds& Interval, AVR::Concepts::AtMega_8  MCU>
    struct SystemTimer<Number, Interval, MCU> {
        using mcu_timer_type = typename TimerParameter<Number, MCU>::mcu_timer_type;
        using tccra_type = typename TimerParameter<Number, MCU>::ta;        
        using tccrb_type = typename TimerParameter<Number, MCU>::tb;        
        using value_type  = typename TimerParameter<Number, MCU>::value_type;        
        using interrupts_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_type;
        using flags_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_flags_type;
        
        static constexpr auto mcuTimer = TimerParameter<Number, MCU>::mcuTimer;
        static constexpr auto mcuInterrupts = TimerParameter<Number, MCU>::mcuInterrupts;
        
        static constexpr auto intervall = Interval;
        static constexpr auto frequency = uint16_t{1} / Interval;
        static constexpr auto tsd = AVR::Util::Timer::calculate<mcu_timer_type, Number>(frequency);
        static_assert(tsd, "falscher wert für p");
        
        template<uint16_t PreScale>
        inline static void prescale() {
            constexpr tccrb_type bits = AVR::Util::Timer::bitsFrom<PreScale>(MCU::Timer8Bit::template PrescalerBits<Number>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcuTimer()->tccrb.template set<bits>();
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
        }
        
        inline static void init() {
//            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
//            prescale<tsd.prescaler>();
//            *mcuTimer()->ocra = tsd.ocr - 1;
        }

        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
//            if (mcuInterrupts()->tifr.template isSet<flags_type::ocfa>()) {
//                f();
//                mcuInterrupts()->tifr.template reset<flags_type::ocfa>(); // reset
//            } 
        }
    };
}
