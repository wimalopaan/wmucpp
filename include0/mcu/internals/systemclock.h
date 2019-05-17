#pragma once

#include <std/chrono>
#include <etl/print.h>

#include "timer.h"

namespace AVR::Util {
    namespace detail {
        template<typename T>
        struct isDuration : std::false_type {};
        
        template<typename T, typename R>
        struct isDuration<std::chrono::duration<T, R>> : std::true_type {};
    }
    
    template<typename T>
    inline constexpr bool is_duration_v = detail::isDuration<std::remove_cv_t<std::remove_reference_t<T>>>::value;
    
    template<typename T>
    concept bool Duration = is_duration_v<T>;
}

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;
    
    template<int Number, const auto& Interval, typename MCU = DefaultMcuType>
    requires AVR::Util::is_duration_v<decltype(Interval)>
    struct SystemTimer {
        using mcu_timer_type = mcu_timer_t<Number>;
        using tccra_type = typename TimerParameter<Number, MCU>::ta;        
        using tccrb_type = typename TimerParameter<Number, MCU>::tb;        
        using value_type  = typename TimerParameter<Number, MCU>::value_type;        
        using interrupts_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_type;
        using flags_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_flags_type;
        
        static constexpr auto mcu_timer = TimerParameter<Number, MCU>::mcu_timer;
        static constexpr auto mcu_timer_interrupts = TimerParameter<Number, MCU>::mcu_timer_interrupts;

        using intervall_type = std::remove_cv_t<std::remove_reference_t<decltype(Interval)>>;
        
        static constexpr auto intervall = Interval;
        static constexpr auto frequency = uint16_t{1} / Interval;
        static constexpr auto tsd = AVR::Util::Timer::calculate<Number>(frequency);
        static constexpr auto exact_intervall = duration_cast<milliseconds>(uint16_t{1} / tsd.f);
        
//                decltype(exact_intervall)::_;
        
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
            constexpr tccrb_type bits = AVR::Util::Timer::bitsFrom<PreScale>(mcu_timer_type::template PrescalerBits<Number>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcu_timer()->tccrb.template set<bits>();
            mcu_timer()->tccra.template set<tccra_type::wgm1>();
        }
        
        inline static void init() {
            mcu_timer()->tccra.template set<tccra_type::wgm1>();
            prescale<tsd.prescaler>();
            *mcu_timer()->ocra = tsd.ocr - 1;
        }
        
        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            if (mcu_timer_interrupts()->tifr.template isSet<flags_type::ocfa>()) {
                f();
                mcu_timer_interrupts()->tifr.template reset<flags_type::ocfa>(); // reset
            } 
        }
    };
    
    template<const auto& Interval, AVR::Concepts::AtMega_8  MCU>
    requires AVR::Util::is_duration_v<decltype(Interval)>
    struct SystemTimer<0, Interval, MCU> {
        inline static constexpr uint8_t Number = 0;
        using mcu_timer_type = typename TimerParameter<Number, MCU>::mcu_timer_type;
        using tccr_type = typename TimerParameter<Number, MCU>::ta;        
        using value_type  = typename TimerParameter<Number, MCU>::value_type;        
        using interrupts_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_type;
        using flags_type = typename TimerParameter<Number, MCU>::mcu_timer_interrupts_flags_type;
        
        static constexpr auto mcu_timer = TimerParameter<Number, MCU>::mcu_timer;
        static constexpr auto mcu_timer_interrupts = TimerParameter<Number, MCU>::mcu_timer_interrupts;
        
        using intervall_type = std::remove_cv_t<std::remove_reference_t<decltype(Interval)>>;

        static constexpr auto intervall = Interval;
        static constexpr auto frequency = uint16_t{1} / Interval;
        static constexpr auto tsd = AVR::Util::Timer::calculate<Number>(frequency);
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
            mcu_timer()->tccr.template set<bits>();
        }
        
        inline static void init() {
            prescale<tsd.prescaler>();
        }
        
        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            if (mcu_timer_interrupts()->tifr.template isSet<flags_type::tov0>()) {
                f();
                mcu_timer_interrupts()->tifr.template reset<flags_type::tov0>(); // reset
            } 
        }
    };
    
    template<const auto& Interval, AVR::Concepts::AtMega_8  MCU>
    requires AVR::Util::is_duration_v<decltype(Interval)>
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
        
        using intervall_type = std::remove_cv_t<std::remove_reference_t<decltype(Interval)>>;

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
