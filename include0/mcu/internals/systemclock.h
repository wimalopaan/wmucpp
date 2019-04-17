#pragma once

#include <std/chrono>

#include "groups.h"

template<auto T>
struct static_print {
    std::integral_constant<uint16_t, T> v;
    using type = typename decltype(v)::_;
};

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
        static constexpr auto exact_intervall = duration_cast<milliseconds>(uint16_t{1} / tsd.f);
        
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
    
    template<const std::chrono::milliseconds& Interval, AVR::Concepts::AtMega_8  MCU>
    struct SystemTimer<0, Interval, MCU> {
        inline static constexpr uint8_t Number = 0;
        using mcu_timer_type = typename TimerParameter<Number, MCU>::mcu_timer_type;
        using tccr_type = typename TimerParameter<Number, MCU>::ta;        
        using value_type  = typename TimerParameter<Number, MCU>::value_type;        
        using interrupts_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_type;
        using flags_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_flags_type;
        
        static constexpr auto mcuTimer = TimerParameter<Number, MCU>::mcuTimer;
        static constexpr auto mcuInterrupts = TimerParameter<Number, MCU>::mcuInterrupts;
        
        static constexpr auto intervall = Interval;
        static constexpr auto frequency = uint16_t{1} / Interval;
        static constexpr auto tsd = AVR::Util::Timer::calculate<mcu_timer_type, Number>(frequency);
        static_assert(tsd, "falscher wert für p");
        static constexpr auto exact_intervall = duration_cast<milliseconds>(uint16_t{1} / tsd.f);
        
//        decltype(exact_intervall)::_;
        
        using _pre = std::integral_constant<uint16_t, tsd.prescaler>;        
        using _ocr = std::integral_constant<uint16_t, tsd.ocr>;        
        using _freq = std::integral_constant<uint16_t, tsd.f.value>;        
        using _int = std::integral_constant<uint16_t, exact_intervall.value>;        
        
//        _pre::_;
//        _ocr::_;
//        _freq::_;
//        _int::_;
        
        template<uint16_t PreScale>
        inline static void prescale() {
            constexpr tccr_type bits = AVR::Util::Timer::bitsFrom<PreScale>(mcu_timer_type::template PrescalerBits<Number>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcuTimer()->tccr.template set<bits>();
        }
        
        inline static void init() {
            prescale<tsd.prescaler>();
        }

        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            if (mcuInterrupts()->tifr.template isSet<flags_type::tov0>()) {
                f();
                mcuInterrupts()->tifr.template reset<flags_type::tov0>(); // reset
            } 
        }
    };
    
    template<const std::chrono::milliseconds& Interval, AVR::Concepts::AtMega_8  MCU>
    struct SystemTimer<2, Interval, MCU> {
        inline static constexpr uint8_t Number = 2;
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
        static constexpr auto exact_intervall = duration_cast<milliseconds>(uint16_t{1} / tsd.f);

//        decltype(exact_intervall)::_;
        
        using _ocr = std::integral_constant<uint16_t, tsd.ocr>;        
        
        template<uint16_t PreScale>
        inline static void prescale() {
            constexpr auto bits = AVR::Util::Timer::bitsFrom<PreScale>(mcu_timer_type::template PrescalerBits<Number>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcuTimer()->tccr.template set<bits>();
        }
        
        inline static void init() {
            prescale<tsd.prescaler>();
            mcuTimer()->tccr.template add<mcu_timer_type::TCCR::wgm1>();
            *mcuTimer()->ocr = tsd.ocr - 1;
        }

        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            if (mcuInterrupts()->tifr.template isSet<flags_type::ocf2>()) {
                f();
                mcuInterrupts()->tifr.template reset<flags_type::ocf2>(); // reset
            } 
        }
    };
}
