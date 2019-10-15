#pragma once

#include <math.h>

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/port.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/adcomparator.h>
#include <mcu/internals/capture.h>
#include <mcu/internals/pwm.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <etl/fixedpoint.h>
#include <etl/output.h>
#include <etl/control.h>

#include <std/chrono>

namespace BLDC {
    using namespace AVR;
    using namespace External::Units;
    using namespace External::Units::literals;
    using namespace std::literals::chrono;
    
    namespace Sine9 {
        
        using fpq10 = etl::FixedPoint<int16_t, 10>;
        using fpq8 = etl::FixedPoint<int16_t, 8>;
        using scale_type = etl::FixedPoint<uint8_t, 7>;
        static inline constexpr uint16_t SpeedMax = 1000;
        using speed_t = etl::uint_ranged<uint16_t, 0, SpeedMax>;
        
        static inline Control::PID currentPid{fpq10{0.01}, fpq10{0.005}, fpq10{0.0}, 100, 30};
        static inline Control::PID periodPid{fpq10{0.1}, fpq10{0.005}, fpq10{0.1}, 100, 200};
        
        static inline scale_type mScaleStart{0.3};
        static inline volatile scale_type mScale = mScaleStart;
        
        struct SineCurrent {
            inline explicit SineCurrent(uint16_t start, uint16_t end, uint16_t max) : mStart{start}, mEnd{end}, mMax{max} {
                assert(mEnd >= mStart);
                assert(mMax >= mStart);
                mDiff = mEnd - mStart;
            }
            inline const uint16_t& start() const {
                return mStart;
            }
            inline const uint16_t& diff() const {
                return mDiff;
            }
            inline void end(const uint16_t& v) {
                assert(v >= mStart);
                mEnd = v;
                mDiff = mEnd - mStart;
            }
            inline const uint16_t& end() const {
                return mEnd;
            }
            inline void max(const uint16_t& v) {
                assert(v >= mStart);
                mMax = v;
            }
            inline const uint16_t& max() const {
                return mMax;
            }
        private:
            uint16_t mStart = 0;
            uint16_t mEnd   = 0;
            uint16_t mMax   = 0;
            uint16_t mDiff  = mEnd - mStart;
        };
        
        struct SineConfig {
            // Prüfungen einbauen
            inline const speed_t& dead() const {
                return mS_Dead;
            }
            inline const speed_t& norm() const {
                return mS_Norm;
            }
            inline const speed_t& stat() const {
                return mS_Static;
            }
            inline const speed_t& sine() const {
                return mS_Sine;
            }
            inline const speed_t& regulated() const {
                return mS_Reg;
            }
            inline const speed_t& regStaticDiff() const {
                return mS_RegStaticDiff;
            }
            inline const speed_t& upperNormDiff() const {
                return mS_UpperNormDiff;
            }
        private:
            speed_t mS_Dead = 2;
            speed_t mS_Static = 10;
            speed_t mS_Sine = 20;
            speed_t mS_Reg  = 40;
            speed_t mS_Norm = 50;
            speed_t mS_RegStaticDiff = mS_Reg - mS_Static;
            speed_t mS_UpperNormDiff = speed_t::Upper - mS_Norm;
        };
        
        struct Config {
            inline uint16_t speedToCurrent(const speed_t& s) {
                return current.start() + (((int32_t(z2) * s.toInt()) >> shift) - c);
            }            
            
            static inline constexpr uint8_t shift = 8;
            static inline constexpr uint16_t factor = (1 << shift);
            
            SineCurrent current{120, 150, 135};
            SineConfig sine;
            
            int16_t z2 = (int32_t(current.diff()) * factor) / sine.regStaticDiff();
            int16_t c = (int32_t(current.diff()) * sine.stat().toInt()) / sine.regStaticDiff();
        };
        
        static inline Config config;
        
        using period_t = etl::uint_ranged<uint16_t, 1000, 20000>;
        
        inline static constexpr std::array<uint8_t, 6> sinePhaseLength{100, 50, 25, 12, 6, 3};
        using sine_table_number_t = etl::uint_ranged<uint8_t, 0, sinePhaseLength.size() - 1>;            
        
        inline static std::array<speed_t, sinePhaseLength.size()> sineSpeeds{};
        inline static uint16_t mP_Dead = 20000;
        
        inline static constexpr auto speedToSine(speed_t s) {
            for(uint8_t i = 1; i < sineSpeeds.size(); ++i) {
                if ((s >= sineSpeeds[i - 1]) && (s < sineSpeeds[i])) {
                    auto p = mP_Dead - (uint32_t{mP_Dead} * (s - sineSpeeds[i - 1])) / (2 * (sineSpeeds[i] - sineSpeeds[i - 1]));
                    return std::pair<period_t, sine_table_number_t>{p, i - 1};
                }
            }
            return std::pair<period_t, sine_table_number_t>{};
        }
        
        template<typename RotTimer, typename ComTimer, typename PWM, typename Commuter, typename AC, typename CSensor, typename Dbg = void>
        struct Controller {
            
            enum class State : uint8_t {Off, 
                                        SineStatic = 10, SineCurrentRegulated, SineRegulatedWait,
                                        PeriodRegulatedStart = 20, PeriodRegulated, PeriodRegulatedDown,
                                        ClosedLoopLow = 30, ClosedLoopMedium, ClosedLoopHigh, ClosedLoopError};
            
            struct SpeedValue {
                inline constexpr SpeedValue() = default;
                inline constexpr SpeedValue(const speed_t& s) : mActualSpeed{s}, mActualTimerValues{speedToSine(s)} {}
                inline const speed_t& speed() const {
                    return mActualSpeed;
                }
                inline const period_t& period()  const {
                    return mActualTimerValues.first;
                }
                inline const sine_table_number_t& sineTable() const {
                    return mActualTimerValues.second;
                }
            private:
                speed_t mActualSpeed{0};
                std::pair<period_t, sine_table_number_t> mActualTimerValues;
            };
            
            constinit inline static SpeedValue speed_value;
            
            inline static constexpr auto sineTableLengths = []{
                std::array<uint16_t, sinePhaseLength.size()> l;
                for(uint8_t i = 0; i < l.size(); ++i) {
                    l[i] = sinePhaseLength[i] * 6;
                }
                return l;
            }();
            
            static inline constexpr uint16_t PwmMax = 200;
            using value_type = etl::typeForValue_t<PwmMax>;
            
            using scale_type = etl::FixedPoint<uint8_t, 7>;
            
            static inline constexpr auto sineTableGenerator = []<uint16_t Size, uint16_t Max = 1000, uint16_t Shift = 0>(std::integral_constant<uint16_t, Size>, 
                                                                                                                         std::integral_constant<uint16_t, Max> = std::integral_constant<uint16_t, Max>{},
                                                                                                                         std::integral_constant<uint16_t, Shift> = std::integral_constant<uint16_t, Shift>{}){
                                                                                                                      using value_type = etl::typeForValue_t<Max>;        
                                                                                                                      std::array<value_type, Size> t;
                                                                                                                      for(uint16_t i = 0; i < t.size(); ++i) {
                auto k = (i + Shift) % t.size();
                t[k] = (Max / 2) * (cos((i * 2 * M_PI) / t.size()) + 1.0);
            }
            return t;
        };
        
        struct ComTimerHandler : AVR::IsrBaseHandler<typename ComTimer::interrupt_type>{
            inline static etl::uint_ranged<uint8_t, 0, sinePhaseLength.size() - 1> mActiveSineTable = 0;
            
            template<uint8_t Table>
            struct SetSine {
                static_assert(Table < sineTableLengths.size());
                
                // shifted LUTs erzeugen
                static constexpr auto sineTableLength = sineTableLengths[Table];
                static constexpr auto sine_table1 = sineTableGenerator(std::integral_constant<uint16_t, sineTableLength>{}, 
                                                                       std::integral_constant<uint16_t, PwmMax>{});
                using size_type = decltype(sine_table1)::size_type;
                using value_type = decltype(sine_table1)::value_type;
                static inline constexpr size_type shift = sine_table1.size() / 3;
                static constexpr auto sine_table2 = sineTableGenerator(std::integral_constant<uint16_t, sineTableLength>{}, 
                                                                       std::integral_constant<uint16_t, PwmMax>{},
                                                                       std::integral_constant<uint16_t, shift>{});
                static constexpr auto sine_table3 = sineTableGenerator(std::integral_constant<uint16_t, sineTableLength>{}, 
                                                                       std::integral_constant<uint16_t, PwmMax>{},
                                                                       std::integral_constant<uint16_t, 2 * shift>{});
                
                using index_type = etl::uint_ranged_circular<size_type, 0, sine_table1.size() - 1>;
                
                static inline /*volatile */ index_type index{}; 
                
                inline static void resetLeft() {
                    index = index + shift / 6;
                }
                
                inline static void setSine() {
                    //                    std::array<value_type, 3> v{sine_table1[index.toInt()], sine_table1[index.template leftShift<shift>().toInt()], sine_table1[index.template leftShift<2 * shift>().toInt()]};
                    size_type i = index.toInt();
                    std::array<value_type, 3> v{sine_table1[i], sine_table2[i], sine_table3[i]};
                    
                    index.increment();
                    if (index == 0) {
                        if (mState == State::PeriodRegulatedStart) {
                            mState = State::PeriodRegulated;
                            
                            ComTimer::template enableInterrupts<false>();
                            AC::template enableInterrupts<true>();
                            
                            Commuter::set(typename Commuter::index_type{0});
                            PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                            
                            return;
                        }
                        else {
                            mActiveSineTable = speed_value.sineTable();
                        }
                    }
                    PWM::template duty<AVR::PWM::WO<0>>(v[0] * mScale);
                    PWM::template duty<AVR::PWM::WO<1>>(v[1] * mScale);
                    PWM::template duty<AVR::PWM::WO<2>>(v[2] * mScale);
                }
                inline constexpr auto operator()() {
                    return setSine();
                } 
            };
            
            static inline void isr() { // 5,2µs
                ComTimer::onInterrupt([&]{
                    etl::select_t<SetSine>(mActiveSineTable);
                });
            }
            
            inline static void resetLeft() {
                mActiveSineTable = speed_value.sineTable();
                switch(mActiveSineTable) {
                case 0:
                    SetSine<0>::resetLeft();
                    break;
                case 1:
                    SetSine<1>::resetLeft();
                    break;
                case 2:
                    SetSine<2>::resetLeft();
                    break;
                case 3:
                    SetSine<3>::resetLeft();
                    break;
                case 4:
                    SetSine<4>::resetLeft();
                    break;
                case 5:
                    SetSine<5>::resetLeft();
                    break;
                }
            }
            
        };
        
        struct AcHandler : AVR::IsrBaseHandler<AVR::ISR::AdComparator<0>::Edge>{
            static inline void isr() { // 12µs (7,7µs ohne delay-for-loop (mDelay = 0)) (ohne cli/sei)
                AC::onInterrupt([&](){
                    uint8_t c = 0;
                    for(uint8_t i = 0; i < mMaxDelay; ++i) {
                        if (!(AC::get() ^ AC::get())) {
                            if(++c > mDelay) {
                                break;
                            }
                        }
                        else {
                            c = 0;
                        }
                    }
                    
//                    Commuter::template OnStateOrNext<0>([&](bool /*reverse*/) {
//                        mActualPeriodLoop = RotTimer::restart(); 
//                        if (mState == State::SineRegulatedWait) {
//                            AC::template enableInterrupts<false>();
//                            ComTimer::template enableInterrupts<true>();
//                            mState = State::SineCurrentRegulated;
                            
//                            ComTimerHandler::resetLeft();                            
//                            //                            etl::select_t<ComTimerHandler::template SetSine>(speed_value.sineTable());
                            
//                            Commuter::template pwm<0>();
//                            Commuter::template pwm<1>();
//                            Commuter::template pwm<2>();
//                        }
//                    });
                    
                    Commuter::next();
                    Commuter::template onState<0>([&] {
                        mActualPeriodLoop = RotTimer::restart(); 
                        if (mState == State::SineRegulatedWait) {
                            AC::template enableInterrupts<false>();
                            ComTimer::template enableInterrupts<true>();
                            mState = State::SineCurrentRegulated;
                            
                            ComTimerHandler::resetLeft();                            
                            etl::select_t<ComTimerHandler::template SetSine>(speed_value.sineTable());
                            
                            Commuter::template pwm<0>();
                            Commuter::template pwm<1>();
                            Commuter::template pwm<2>();
                        }
                    });
                });
            };
            inline static uint8_t mDelay = 0;
            inline static constexpr uint8_t mMaxDelay = 10;
        };
        
        inline static void init() {
            CSensor::init();
            AC::init();
            AC::template enableInterrupts<false>();
            Commuter::init();
            RotTimer::init();
            ComTimer::init();
            ComTimer::period2(speed_value.period());
            ComTimer::template enableInterrupts<false>();
            
            updateSineSpeeds();
        }
        
        inline static void periodic() {
        }
        
        static inline uint16_t npwm = 0;
        
        //        using fpq10 = etl::FixedPoint<int16_t, 10>;
        //        using fpq8 = etl::FixedPoint<int16_t, 8>;
        
        //        static inline Control::PID currentPid{fpq10{0.1}, fpq10{0.005}, fpq10{0.0}, 100, 30};
        //        static inline Control::PID periodPid{fpq10{0.5}, fpq10{0.005}, fpq10{0.1}, 100, 200};
        
        inline static void ratePeriodic() {
            if ((mActualPeriodLoop < (2 * mActualPeriodEstimate)) && (mActualPeriodLoop > (mActualPeriodEstimate / 2))) {
                mActualPeriodEstimate = (uint32_t{3} * mActualPeriodEstimate + mActualPeriodLoop) / 4;
            }
            
            switch(mState) {
            case State::Off:
                if (speed_value.speed() >= config.sine.dead()) {
                    AC::template enableInterrupts<false>();
                    ComTimer::template enableInterrupts<true>();
                    Commuter::template pwm<0>();
                    Commuter::template pwm<1>();
                    Commuter::template pwm<2>();
                    mScale = mScaleStart;
                    mState = State::SineStatic;
                }
                else {
                    Commuter::off();
                    AC::template enableInterrupts<false>();
                    ComTimer::template enableInterrupts<false>();
                }
                break;
            case State::SineStatic:
                if (speed_value.speed() >= config.sine.stat()) {
                    mState = State::SineCurrentRegulated;
                    mScale = mScaleStart;
                    currentPid.reset();
                }
                else if (speed_value.speed()< config.sine.dead()) {
                    mState = State::Off;
                }
                else {
                    ComTimer::period2(speed_value.period()); 
                }
                break;
            case State::SineCurrentRegulated:
                if (speed_value.speed() < config.sine.stat()) {
                    mScale = mScaleStart;
                    mState = State::SineStatic;
                }
                else if (speed_value.speed() >= config.sine.regulated()) {
                    mActualPeriodEstimate = desiredRtcPeriod();
                    mActualPeriodLoop = 0;
                    mActualPwm = PwmMax * mScale * mPwmAdjust;
                    periodPid.reset();
                    mState = State::PeriodRegulatedStart;
                }
                else {
                    
                    // mit debug vermessen
                    
                    ComTimer::period2(speed_value.period()); 
                    
                    // Division mit Multiplikation und Shift (berechnen, wenn Parameter geändert werden)
                    //                    uint16_t mCurrentSine = mCurrent.start() + (int32_t(mCurrent.diff()) * (speed_value.speed() - sineConfig.stat())) / sineConfig.regStaticDiff();
                    //                    uint16_t mCurrentSine = config.current.start() + (int32_t(config.current.diff()) * (speed_value.speed() - config.sine.stat())) / config.sine.regStaticDiff();
                    //                    uint16_t mCurrentSine = config.current.start() 
                    //                                            + (int32_t(config.current.diff()) * speed_value.speed().toInt()) / config.sine.regStaticDiff()
                    //                                            - (int32_t(config.current.diff()) * config.sine.stat().toInt()) / config.sine.regStaticDiff();
                    
                    uint16_t mCurrentSine = config.speedToCurrent(speed_value.speed());
                    
                    mCurrentSine = std::clamp(mCurrentSine, config.current.start(), config.current.max());
                    
                    auto cv = currentPid.correctionValue(CSensor::value(), mCurrentSine);
                    mScale = mScaleStart - scale_type{0.1} * cv;
                }
                break;
            case State::PeriodRegulated:
                if (speed_value.speed() < config.sine.sine()) {
                    ComTimer::period2(speed_value.period()); 
                    mState = State::SineRegulatedWait;
                    currentPid.reset();
                }
                else if (speed_value.speed() >= config.sine.norm()) {
                    mState = State::ClosedLoopLow;
                    mActualPwm = npwm;
                }
                else {
                    auto cvp = periodPid.correctionValue(mActualPeriodEstimate, desiredRtcPeriod());
                    
                    npwm = mActualPwm + cvp * scale_type{1.0};                
                    
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(npwm);
                }
                break;
            case State::ClosedLoopLow:
                if (speed_value.speed() <= config.sine.regulated()) {
                    mActualPwm = npwm;
                    mState = State::PeriodRegulated;
                    periodPid.reset();
                }
                else {
                    npwm = mActualPwm + (int32_t(PWM::max() - mActualPwm) * (int16_t(speed_value.speed()) - int16_t(config.sine.norm()))) / config.sine.upperNormDiff();
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(npwm);
                    AC::hysterese(AC::hyst_t::hyst_large);
                }
                break;
            case State::ClosedLoopMedium:
                npwm = mActualPwm + (int32_t(PWM::max() - mActualPwm) * (int16_t(speed_value.speed()) - int16_t(config.sine.norm()))) / config.sine.upperNormDiff();
                PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(npwm);
                AC::hysterese(AC::hyst_t::hyst_medium);                    
                if (mActualPeriodEstimate < mPeriodHystSmall) {
                    mState = State::ClosedLoopHigh;
                }
                else if (mActualPeriodEstimate > mPeriodHystLarge) {
                    mState = State::ClosedLoopLow;
                } 
                break;
            case State::ClosedLoopHigh:
                npwm = mActualPwm + (int32_t(PWM::max() - mActualPwm) * (int16_t(speed_value.speed()) - int16_t(config.sine.norm()))) / config.sine.upperNormDiff();
                PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(npwm);
                AC::hysterese(AC::hyst_t::hyst_small);                    
                if (mActualPeriodEstimate > mPeriodHystMedium) {
                    mState = State::ClosedLoopHigh;
                }
                break;
            default:
                break;
            }
        }
        inline static uint16_t desiredRtcPeriod() {
            return (speed_value.period().toInt() * uint32_t{sineTableLengths[speed_value.sineTable()]}) / (Project::Config::fMcu / Project::Config::fRtc);
        }
        
        inline static void speed(speed_t s) {
            speed_value = SpeedValue{s};
        }
        
        inline static void off() {
            currentPid.reset();            
            periodPid.reset();            
            mScale = scale_type{0.3};
            mState = State::Off;
            Commuter::off();
            AC::template enableInterrupts<false>();
            ComTimer::template enableInterrupts<false>();
        }
        
        inline static void pwmInc() {
            mActualPwm = mActualPwm + 1;
            PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
        }
        inline static void pwmDec() {
            mActualPwm = mActualPwm - 1;
            PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
        }
        
        
        inline static volatile uint16_t mActualPwm = 0;
        
        // zusammenfassen, auch mit Schäzung
        inline static volatile uint16_t mActualPeriodLoop{0};
        inline static uint16_t mActualPeriodEstimate{0};
        
        inline static volatile State mState = State::Off;
        
        // struct zusammenfassen
        inline static uint16_t mPeriodHystLarge  = 500;
        inline static uint16_t mPeriodHystMedium = 300;
        inline static uint16_t mPeriodHystSmall  = 200;
        
        inline static scale_type mPwmAdjust{0.8};
        
        inline static void updateSineSpeeds() {
            constexpr uint8_t N = sinePhaseLength.size();
            constexpr uint8_t ii = (N * (N + 1) * (2 * N + 1)) / 6;
            const uint16_t d = (config.sine.norm() - config.sine.norm()) / ii + 1;
            sineSpeeds[0] = d;
            for(uint8_t i = 1; i < N; ++i) {
                sineSpeeds[i] = sineSpeeds[i - 1] + d * (1 << i);
            }
            sineSpeeds[N - 1] = config.sine.norm();
        }
    };
}
}
