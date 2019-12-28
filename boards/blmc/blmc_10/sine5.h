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

#include <std/chrono>

namespace Control {
    template<typename ValueType = uint16_t, typename T = etl::FixedPoint<int16_t, 10>>
    struct PID {
        using value_type = ValueType;
        using error_type = std::make_signed_t<value_type>;
        using factor_type = T;
        
        constexpr PID(const factor_type& p, const factor_type& i, const factor_type& d, const value_type& maxError, const value_type& maxCv) : 
            pFactor(p), iFactor(i), dFactor(d), maxError(maxError), maxCorrection{maxCv} {}
        
        inline error_type correctionValue(const value_type& actual, const value_type& setPoint) {
            av = actual;
            error_type error = std::clamp(actual - setPoint, -maxError, maxError);
            ev = error;
            error_type pterm = error * pFactor;
            pt = pterm;
            di = iFactor * error;
            if (di > factor_type{0.0}) {
                if (integral < (integral.max() - di)) {
                    integral += di;
                }
            }
            else if (di < factor_type{0.0}) {
                if (integral > (integral.min() - di)) {
                    integral += di;
                }
            }
            
            error_type dterm = (actual - prevValue) * dFactor;    
            prevValue = actual;
            
            return cv = std::clamp(pterm + integral.integer() + dterm, -maxCorrection, maxCorrection);
        }
        inline void reset() {
            prevValue = value_type{0};
            integral = factor_type{0.0};
        }
//    private:
        error_type cv;
        error_type pt;
        value_type prevValue{0};
        value_type av;
        value_type ev;
        factor_type di;
        factor_type integral{0.0};
        const factor_type pFactor;
        const factor_type iFactor;
        const factor_type dFactor;
        const value_type maxError{0};
        const value_type maxCorrection{0};
    };
    
    template<typename Stream, typename V, typename F> 
    constexpr inline void out_impl(const PID<V, F>& pid) {
        etl::out<Stream>("in: "_pgm, pid.av, " er: "_pgm, pid.ev, " pt: "_pgm, pid.pt, " it:"_pgm, pid.integral, " cv:"_pgm, pid.cv);
    }
}


namespace BLDC {
    using namespace AVR;
    using namespace External::Units;
    using namespace External::Units::literals;
    using namespace std::literals::chrono;
    
    namespace Sine7 {
        
        template<typename RotTimer, typename ComTimer, typename PWM, typename Commuter, typename AC, typename CSensor>
        struct Controller {
            
            using all_channels = typename PWM::all_channels;
            
            enum class State : uint8_t {Off, SineStatic, SineRegulated, RegulatedStart, Regulated, ClosedLoopStart, PreClosedLoop, 
                                        ClosedLoopLow, ClosedLoopMedium, ClosedLoopHigh, ClosedLoopError};
            
            inline static constexpr std::array<uint8_t, 6> sinePhaseLength{100, 50, 25, 12, 6, 3};

            
            static inline constexpr uint16_t SpeedMax = 1000;
            using speed_t = etl::uint_ranged<uint16_t, 0, SpeedMax>;
            
            inline static std::array<speed_t, sinePhaseLength.size()> sineSpeeds{};
            
            static inline constexpr uint16_t PwmMax = 200;
            using value_type = etl::typeForValue_t<PwmMax>;
            
            using scale_type = etl::FixedPoint<uint8_t, 7>;
            
            static inline constexpr auto gen = []<uint16_t Size, uint16_t Max = 1000>(std::integral_constant<uint16_t, Size>, 
                                                                                      std::integral_constant<uint16_t, Max> = std::integral_constant<uint16_t, Max>{}){
                                                                                using value_type = etl::typeForValue_t<Max>;        
                                                                                std::array<value_type, Size> t;
                                                                                for(uint16_t i = 0; i < t.size(); ++i) {
                                                                                           t[i] = (Max / 2) * (cos((i * 2 * M_PI) / t.size()) + 1.0);
                                                                                }
                                                                                return t;
                                                                        };
        
        
        struct AcHandler : AVR::IsrBaseHandler<AVR::ISR::AdComparator<0>::Edge>{
            static inline void isr() {
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
                    Commuter::next();
                    Commuter::template onState<0>([&] {
                        mActualPeriodLoop = RotTimer::restart(); 
                    });
                });
            };
            inline static constexpr uint8_t mDelay = 4;
            inline static constexpr uint8_t mMaxDelay = 10;
        };
        
        inline static volatile State mState = State::Off;
        
        struct ComTimerHandler : AVR::IsrBaseHandler<typename ComTimer::interrupt_type>{
            static inline void isr() {
                ComTimer::onInterrupt([&]{
                    etl::select_t<SetSine>(mActiveSineTable);
                });
            }
        };
        
        inline static void init() {
            CSensor::init();
            AC::init();
            AC::template enableInterrupts<false>();
            Commuter::init();
            RotTimer::init();
            ComTimer::init();
            ComTimer::period(mActualPeriod);
            ComTimer::template enableInterrupts<false>();
            
            updateSineSpeeds();
        }
        
        template<uint16_t Size>
        inline static void setSine() {
            static constexpr auto sine_table = gen(std::integral_constant<uint16_t, Size>{}, std::integral_constant<uint16_t, PwmMax>{});
            using size_type = decltype(sine_table)::size_type;
            using value_type = decltype(sine_table)::value_type;
            constexpr size_type shift = sine_table.size() / 3;
            
            static etl::uint_ranged_circular<size_type, 0, Size - 1> index{}; 
            
            std::array<value_type, 3> v{sine_table[index], 
                        sine_table[index.template leftShift<shift>()], 
                        sine_table[index.template leftShift<2*shift>()]};
            
            if (++index == 0) {
                if (mState == State::RegulatedStart) {
                    
                    ComTimer::template enableInterrupts<false>();
             
                    AC::template enableInterrupts<true>();
                    
                    Commuter::set(typename Commuter::state_type{1});
                    
                    PWM::template duty<all_channels>(mActualPwm);
                    
                    mState = State::Regulated;
                    return;
                }
                else {
                    mActiveSineTable = mActualSineTable;
                }
            }
            
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(v[0] * mScale);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(v[1] * mScale);
            PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(v[2] * mScale);
            
//            return v;
        }
        
        inline static void enableSineMode(bool v) {
            if (v) {
                
            }
            else {
                
            }
        }
        
        template<uint8_t Table>
        struct SetSine {
            inline constexpr auto operator()() {
                return setSine<6 * sinePhaseLength[Table]>();
            } 
        };
        
        inline static void periodic() {
        }

        static inline uint16_t npwm = 0;

        using fpq10 = etl::FixedPoint<int16_t, 10>;
        using fpq8 = etl::FixedPoint<int16_t, 8>;

        static inline Control::PID currentPid{fpq10{0.1}, fpq10{0.005}, fpq10{0.0}, 100, 30};
        static inline Control::PID periodPid{fpq10{0.9}, fpq10{0.1}, fpq10{0.1}, 100, 200};
        
        static inline int16_t cv;
        static inline int16_t cvp;
        
        inline static void ratePeriodic() {
            if ((mActualPeriodLoop < 2 * mActualPeriodEstimate) && (mActualPeriodLoop > mActualPeriodEstimate / 2)) {
                mActualPeriodEstimate = (uint32_t{3} * mActualPeriodEstimate + mActualPeriodLoop) / 4;
            }
            
            switch(mState) {
            case State::Off:
                if (mActualSpeed >= mS_Dead) {
                    mState = State::SineStatic;
                }
                break;
            case State::SineStatic:
                mScale = mScaleStart;
                if (mActualSpeed >= mS_Static) {
                    mState = State::SineRegulated;
                }
                else if (mActualSpeed < mS_Dead) {
                    mState = State::Off;
                } 
                break;
            case State::SineRegulated:
                cv = currentPid.correctionValue(CSensor::value(), mCurrentSine);
                mScale = mScaleStart - scale_type{0.1} * cv;
                if (mActualSpeed < mS_Static) {
                    mState = State::SineStatic;
                }
                if (mActualSpeed > mS_Reg) {
//                if (mDesiredPeriod < mPeriodS0) {
                    mActualPeriodEstimate = mDesiredPeriod;
                    mState = State::RegulatedStart;
                }
                break;
            case State::RegulatedStart:
                break;
            case State::Regulated:
            {
                cvp = periodPid.correctionValue(mActualPeriodEstimate, mDesiredPeriod);
                cvp = 0;
                npwm = mActualPwm + cvp * scale_type{0.4};                
                PWM::template duty<all_channels>(npwm);
            }
                break;
            case State::ClosedLoopLow:
                AC::hysterese(AC::hyst_t::hyst_large);
                if (mActualPeriodEstimate < mPeriodHystMedium) {
                    mState = State::ClosedLoopMedium;
                }
                break;
            case State::ClosedLoopMedium:
                AC::hysterese(AC::hyst_t::hyst_medium);                    
                if (mActualPeriodEstimate < mPeriodHystSmall) {
                    mState = State::ClosedLoopHigh;
                }
                else if (mActualPeriodEstimate > mPeriodHystLarge) {
                    mState = State::ClosedLoopLow;
                } 
                break;
            case State::ClosedLoopHigh:
                AC::hysterese(AC::hyst_t::hyst_small);                    
                if (mActualPeriodEstimate > mPeriodHystMedium) {
                    mState = State::ClosedLoopHigh;
                }
                break;
            default:
                break;
            }
        }
        

        inline static auto speedToSine(speed_t s) {
            for(uint8_t i = 1; i < sineSpeeds.size(); ++i) {
                if ((s >= sineSpeeds[i - 1]) && (s < sineSpeeds[i])) {
                    auto p = mP_Dead - (uint32_t{mP_Dead} * (s - sineSpeeds[i - 1])) / (2 * (sineSpeeds[i] - sineSpeeds[i - 1]));
                    return std::pair<uint16_t, uint8_t>{uint16_t(p), uint8_t(i - 1)};
                }
            }
            return std::pair<uint16_t, uint8_t>{};
        }

        inline static void speed(speed_t s) {
            
            auto pt = speedToSine(s);            
            
            mActualSpeed = s;
            
            if (pt.first == 0) {
                off();
                mState = State::Off;
                return;
            }
            
            if ((mState != State::Off) && (mState != State::SineStatic) && (mState != State::SineRegulated)) return; 
            
            mActualPeriod = pt.first;
            mActualSineTable = pt.second;
            
            mDesiredPeriod = (mActualPeriod.toInt() * uint32_t{sinePhaseLength[mActualSineTable]} * 6) / (Project::Config::fMcu / Project::Config::fRtc);
            
            mCurrentSine = mCurrentSineStart + (int32_t(mCurrentSineEnd - mCurrentSineStart) * (s - mS_Static)) / (mS_Reg - mS_Static);
            mCurrentSine = std::clamp(mCurrentSine, mCurrentSineStart, mCurrentSineMax);
            
            AC::template enableInterrupts<false>();
            ComTimer::template enableInterrupts<true>();

            ComTimer::period(mActualPeriod); 
            
            Commuter::template pwm<0>();
            Commuter::template pwm<1>();
            Commuter::template pwm<2>();
            
            mActualPwm = PwmMax * mScale * mPwmAdjust;
        }
        
        inline static void off() {
            currentPid.reset();            
            periodPid.reset();            
            mCurrentSine = mCurrentSineStart;
            mScale = scale_type{0.3};
            Commuter::off();
            AC::template enableInterrupts<false>();
        }
        
        inline static void pwmInc() {
            ++mActualPwm;
            PWM::template duty<all_channels>(mActualPwm);
        }
        inline static void pwmDec() {
            --mActualPwm;
            PWM::template duty<all_channels>(mActualPwm);
        }
        
        
        inline static uint16_t mActualPwm = 0;
        inline static volatile etl::uint_ranged<uint16_t, 1000, 40000> mActualPeriod{20000};

        inline static volatile uint16_t mActualPeriodLoop{0};
        inline static volatile uint16_t mActualPeriodEstimate{0};
        
        inline static volatile uint16_t mDesiredPeriod = 0;
        
        inline static volatile etl::uint_ranged<uint8_t, 0, sinePhaseLength.size() - 1> mActualSineTable{};
        inline static volatile etl::uint_ranged<uint8_t, 0, sinePhaseLength.size() - 1> mActiveSineTable{};
        
//        static inline volatile scale_type mScale{0.25};
//        inline static uint16_t mCurrentMax = 119;
//        inline static uint16_t mCurrentMin = 116;
        

        static inline volatile scale_type mScaleStart{0.3};
        static inline volatile scale_type mScale = mScaleStart;
        
        inline static uint16_t mCurrentSineStart = 120;
        inline static uint16_t mCurrentSineEnd   = 150;
        inline static uint16_t mCurrentSineMax   = 135;
        inline static uint16_t mCurrentSine      = mCurrentSineStart;
        
//        inline static uint16_t mPeriodS0 = 2000;
        inline static uint16_t mPeriodS0 = 600;
        inline static uint16_t mPeriodS1 = mPeriodS0 * etl::FixedPoint<uint8_t, 7>{1.2};
        
        inline static uint16_t mPeriodHystLarge  = 500;
        inline static uint16_t mPeriodHystMedium = 300;
        inline static uint16_t mPeriodHystSmall  = 200;
        
        inline static scale_type mPwmAdjust{0.8};

        inline static speed_t mActualSpeed;
        
        inline static speed_t mS_Dead{2};
        inline static speed_t mS_Static{10};
        inline static speed_t mS_Sine{50};
        inline static speed_t mS_Reg{60};
        inline static speed_t mS_Norm{70};

        inline static uint16_t mP_Dead = 10000;
        inline static uint16_t mP_Reg  = mP_Dead / 30;

        inline static void updateSineSpeeds() {
            constexpr uint8_t N = sinePhaseLength.size();
            constexpr uint8_t ii = (N * (N + 1) * (2 * N + 1)) / 6;
            const uint16_t d = (mS_Norm - mS_Dead) / ii + 1;
            sineSpeeds[0] = d;
            for(uint8_t i = 1; i < N; ++i) {
                sineSpeeds[i] = sineSpeeds[i - 1] + d * (1 << i);
            }
            sineSpeeds[N - 1] = mS_Norm;
        }
        
//        template<typename T, typename PID>
//        inline static void out(const PID& pid) {
//            etl::outl<T>("pt: "_pgm, pid.pt, " in:"_pgm, pid.integral, " cv:"_pgm, pid.cv);
//        }
//        template<typename T>
//        inline static void outCPid() {
//            out<T>(currentPid);
//        }        
//        template<typename T>
//        inline static void outPPid() {
//            out<T>(periodPid);
//        }        
    };
}
}
