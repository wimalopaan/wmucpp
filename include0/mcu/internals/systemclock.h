#pragma once

#include <std/chrono>

#include "timer.h"

namespace AVR::Util {
    namespace detail {
        template<typename T>
        struct isDuration : std::false_type {};
        
        template<typename T, typename R>
        struct isDuration<std::chrono::duration<T, R>> : std::true_type {};

    
        template<typename T>
        struct isFrequency : std::false_type {};
        
        template<typename R, typename D>
        struct isFrequency<External::Units::frequency<R, D>> : std::true_type {};
    }
    
    template<typename T>
    inline constexpr bool is_duration_v = detail::isDuration<std::remove_cv_t<std::remove_reference_t<T>>>::value;

    template<typename T>
    inline constexpr bool is_frequency_v = detail::isFrequency<std::remove_cv_t<std::remove_reference_t<T>>>::value;
    
    template<typename T>
    concept bool Duration = is_duration_v<T>;

    template<typename T>
    concept bool Frequency = is_frequency_v<T>;
}

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;

    template<typename CNumber, const auto& Interval, typename MCU = DefaultMcuType>
    struct SystemTimer;
    
    template<AVR::Concepts::ComponentNumber CNumber, const auto& Interval, AVR::Concepts::AtMega MCU>
    requires AVR::Util::is_duration_v<decltype(Interval)>
    struct SystemTimer<CNumber, Interval, MCU> {
        static inline constexpr auto Number = CNumber::value;
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
    
    template<AVR::Concepts::ComponentNumber CNumber, const auto& Interval, AVR::Concepts::AtMega_8  MCU>
    requires (CNumber::value == 0) && AVR::Util::is_duration_v<decltype(Interval)>
    struct SystemTimer<CNumber, Interval, MCU> {
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
    
    template<AVR::Concepts::ComponentNumber CNumber, const auto& Interval, AVR::Concepts::AtMega_8  MCU>
    requires (CNumber::value == 2) && AVR::Util::is_duration_v<decltype(Interval)>
    struct SystemTimer<CNumber, Interval, MCU> final {
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

    template<AVR::Concepts::ComponentSpecifier CNumber, const auto& Interval, AVR::Concepts::At01Series MCU>
    requires (std::is_same_v<AVR::A, typename CNumber::component_type>) && AVR::Util::is_duration_v<decltype(Interval)>
    struct SystemTimer<CNumber, Interval, MCU> {
        using value_type  = uint16_t;  
        
        using ctrla_t = typename MCU::TCA::CtrlA_t;
        using intflags_t = typename MCU::TCA::Intflags_t;
        
        inline static auto constexpr n = CNumber::value;
        
        static constexpr auto mcu_timer = AVR::getBaseAddr<typename MCU::TCA, n>;
        static constexpr auto prescaler_values = MCU::TCA::prescalerValues;

        static constexpr auto intervall = Interval;
//        std::integral_constant<uint16_t, intervall.value>::_;
        
        static constexpr auto frequency = uint16_t{1} / Interval;

        using tsd_type = TimerSetupData<value_type>;
        
        static inline constexpr auto calculate(const hertz& ftimer) {
            auto prescalers = prescalerValues(prescaler_values);
            
            for(const auto& p : etl::sort(prescalers)) { // aufsteigend
                if (p > 0) {
                    const auto tv = (Project::Config::fMcu / ftimer) / p;
                    if ((tv > 0) && (tv < std::numeric_limits<value_type>::max())) {
                        const bool exact = ((Project::Config::fMcu.value / p) % tv) == 0;
                        return tsd_type{p, static_cast<value_type>(tv), Project::Config::fMcu / tv / uint32_t(p), exact};
                    }
                }
            }
            return tsd_type{};
        }
        
        static constexpr auto tsd = calculate(frequency);
        static_assert(tsd, "falscher wert für p");
        static constexpr auto exact_intervall = duration_cast<milliseconds>(uint16_t{1} / tsd.f);
        
//        decltype(exact_intervall)::_;
//        std::integral_constant<uint16_t, exact_intervall.value>::_;        
//        std::integral_constant<uint16_t, tsd.ocr>::_;        
//        std::integral_constant<uint16_t, tsd.prescaler>::_;        
        
        template<uint16_t PreScale>
        inline static void prescale() {
            constexpr auto p = AVR::Util::Timer::bitsFrom<PreScale>(prescaler_values);
            mcu_timer()->ctrla.template set<p | ctrla_t::enable>();
        }
        
        inline static void init() {
            prescale<tsd.prescaler>();
            *mcu_timer()->perbuf = tsd.ocr;
        }
        
        template<etl::Concepts::Callable Callable, auto Flag = MCU::TCA::Intflags_t::ovf>
        inline static void periodic(const Callable& f) {
            mcu_timer()->intflags.template testAndReset<Flag>([&](){
                f();
            });
        }
    };


    template<const auto& Frequency, AVR::Concepts::At01Series MCU>
    requires AVR::Util::is_frequency_v<decltype(Frequency)>
    struct SystemTimer<AVR::Component::Rtc<0>, Frequency, MCU> final {
        using value_type  = uint16_t;  
        
        using Status_t = MCU::Rtc::Status_t;
        
        using ctrla_t = typename MCU::Rtc::CtrlA_t;
        using intflags_t = typename MCU::Rtc::IntFlags_t;
        
        static constexpr auto mcu_rtc = AVR::getBaseAddr<typename MCU::Rtc>;
        static constexpr auto prescaler_values = MCU::Rtc::prescalerValues;

        static constexpr auto intervall = uint16_t{1} / Frequency;
        static constexpr auto frequency = Frequency;

        using tsd_type = TimerSetupData<value_type>;
        
        static inline constexpr auto calculate(const hertz& ftimer) {
            auto prescalers = prescalerValues(prescaler_values);
            
            for(const auto& p : etl::sort(prescalers)) { // aufsteigend
                if (p > 0) {
                    const auto tv = (Project::Config::fRtc / ftimer) / p;
                    if ((tv > 0) && (tv < std::numeric_limits<value_type>::max())) {
                        const bool exact = ((Project::Config::fRtc.value / p) % tv) == 0;
                        return tsd_type{p, static_cast<value_type>(tv), Project::Config::fRtc/ tv / uint32_t(p), exact};
                    }
                }
            }
            return tsd_type{};
        }
        
        static constexpr auto tsd = calculate(frequency);
        static_assert(tsd, "falscher wert für p, die Frequenz ist mit der RTC nicht darstellbar!");
        static constexpr auto exact_intervall = duration_cast<milliseconds>(uint16_t(1) / tsd.f);
        
        static_assert(exact_intervall.value > 0, "no valid intervall");
        
//        std::integral_constant<uint16_t, exact_intervall.value>::_;        
//        std::integral_constant<uint16_t, tsd.ocr>::_;        
//        std::integral_constant<uint16_t, tsd.prescaler>::_;        
//        std::integral_constant<uint16_t, tsd.f.value>::_;        
        
        template<uint16_t PreScale>
        inline static void prescale() {
            constexpr auto p = AVR::Util::Timer::bitsFrom<PreScale>(prescaler_values);
//            std::integral_constant<uint16_t, (uint16_t)p>::_;        
            while(mcu_rtc()->status.template isSet<Status_t::ctrlabusy>()) {}
            mcu_rtc()->ctrla.template set<p | ctrla_t::enable>();
        }

        template<uint16_t Period>
        inline static void period() {
//            std::integral_constant<uint16_t, Period>::_;        
            while(mcu_rtc()->status.template isSet<Status_t::perbusy>()) {}
            *mcu_rtc()->per = Period;          
        }
        
        
        inline static void init() {
            prescale<tsd.prescaler>();
            period<tsd.ocr>();
        }
        
        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            mcu_rtc()->intflags.template testAndReset<intflags_t::ovf>([&]{
                ++mOverflows;
                f();
            });
        }
        inline static const uint16_t& overflows() {
            return mOverflows;
        }  
        inline static uint16_t actual() {
            return (mOverflows * tsd.ocr) + *mcu_rtc()->cnt;
        }  
        private:
        static inline uint16_t mOverflows{0};
    };

//    template<const auto& Frequency, AVR::Concepts::At01Series MCU>
//    requires AVR::Util::is_frequency_v<decltype(Frequency)>
//    struct SystemTimer<AVR::Component::Rtc<0>, Frequency, MCU> final {
//        using value_type  = uint16_t;  
        
//        using rtcStatus_t = MCU::Rtc::Status_t;
//        using Status_t = MCU::Rtc::PitStatus_t;
        
//        using ctrla_t = typename MCU::Rtc::PitCtrlA_t;
//        using intflags_t = typename MCU::Rtc::PitIntFlags_t;
        
//        static constexpr auto mcu_rtc = AVR::getBaseAddr<typename MCU::Rtc>;
//        static constexpr auto prescaler_values = MCU::Rtc::pitPrescalerValues;

//        static constexpr auto intervall = uint16_t{1} / Frequency;
//        static constexpr auto frequency = Frequency;

//        using tsd_type = TimerSetupData<value_type>;
        
//        static inline constexpr auto calculate(const hertz& ftimer) {
//            auto prescalers = prescalerValues(prescaler_values);
            
//            for(const auto& p : etl::sort(prescalers)) { // aufsteigend
//                if (p > 0) {
//                    const auto f = Project::Config::fRtc / p;
//                    if (f == ftimer) {
//                        return tsd_type{p, 1, f, true};
//                    }
//                }
//            }
//            return tsd_type{};
//        }
        
//        static constexpr auto tsd = calculate(frequency);
//        static_assert(tsd, "falscher wert für p, die Frequenz ist mit der RTC nicht darstellbar!");
//        static_assert(tsd.f < External::Units::hertz{1000}, "Frequenz zu hoch");
//        static constexpr auto exact_intervall = duration_cast<milliseconds>(uint16_t{1} / tsd.f);
        
//        static_assert(exact_intervall.value > 0, "no valid intervall");
        
////        std::integral_constant<uint16_t, exact_intervall.value>::_;        
////        std::integral_constant<uint16_t, tsd.ocr>::_;        
////        std::integral_constant<uint16_t, tsd.prescaler>::_;        
////        std::integral_constant<uint16_t, tsd.f.value>::_;        
        
//        template<uint16_t PreScale>
//        inline static void prescale() {
//            constexpr auto p = AVR::Util::Timer::bitsFrom<PreScale>(prescaler_values);
////            std::integral_constant<uint16_t, (uint16_t)p>::_;        
//            while(mcu_rtc()->pitstatus.template isSet<Status_t::ctrlbusy>()) {}
//            mcu_rtc()->pitctrla.template set<p | ctrla_t::enable>();
//        }
        
//        inline static void init() {
//            prescale<tsd.prescaler>();
//        }
        
//        template<etl::Concepts::Callable Callable>
//        inline static void periodic(const Callable& f) {
//            mcu_rtc()->pitintflags.template testAndReset<intflags_t::pi>([&]{
//                f();
//            });
//        }
//    };
    
}
