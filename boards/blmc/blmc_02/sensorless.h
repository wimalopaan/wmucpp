#pragma once

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

namespace BLDC {
    using namespace AVR;
    using namespace External::Units;
    using namespace External::Units::literals;
    using namespace std::literals::chrono;
    
    template<typename Capture, typename PWM, typename Com, typename xAdc, typename OL>
    struct Controller {
        typedef typename Capture::value_type timer_value_t;
        typedef typename PWM::value_type pwm_value_t;
        
        static inline constexpr pwm_value_t pwm_max = PWM::max();
        
        struct RampValue {
            timer_value_t tv;
            etl::FixedPoint<uint16_t, 8> pwm;
            
            inline RampValue& operator+=(const RampValue& rhs) {
                tv -= rhs.tv;
                pwm += rhs.pwm;
                return *this;
            }
        };
        
        struct Ramp {
            RampValue start;
            RampValue end;
            RampValue delta;
            uint8_t   steps;
        };
        
        struct RampPoint {
            std::chrono::microseconds time;
            External::Units::percent pvm;
        };
        
        enum class State : uint8_t {Off, Align, Align2, RampUp, ClosedLoop, ClosedLoopComDelay, ClosedLoopCommute, ClosedLoopError};
        
        static inline constexpr uint16_t prescaler = 8;
        static inline constexpr auto fTimer = Project::Config::fMcu / prescaler;
        
        // 190KV / 64mm    
        static inline RampPoint ramp_start = {20000_us, 5_ppc};
        static inline RampPoint ramp_end   = {2000_us, 15_ppc};
        static inline uint8_t ramp_steps = 50;
        
        //    static inline constexpr auto x = expand(5_ppc, pwm_value_t{0}, pwm_max);
        //    std::integral_constant<decltype(x), x>::_;
        
        static inline constexpr auto pwmStart = etl::FixedPoint<uint16_t, 8>{(double)expand(10_ppc, pwm_value_t{0}, pwm_max)};
        
        static inline constexpr uint32_t alignValue2 = 200_ms * fTimer;
        static inline constexpr timer_value_t alignValueCount = alignValue2 / std::numeric_limits<timer_value_t>::module();
        static inline constexpr timer_value_t alignValueLast  = alignValue2 % std::numeric_limits<timer_value_t>::module();
        
        // 1000KV/37mm
        //    static inline RampPoint ramp_start = {20000_us, 3_ppc};
        //    static inline RampPoint ramp_end   = {2000_us, 6_ppc};
        //    static inline uint8_t ramp_steps = 50;
        //    static inline auto pwmStart = FixedPoint<uint16_t, 8>{(double)std::expand(5_ppc, uint8_t{0}, pwm_max)};
        
        inline static RampValue convert(const RampPoint& p) {
            RampValue v;
            v.tv = p.time * fTimer;
            v.pwm = etl::FixedPoint<uint16_t, 8>{(double)expand(p.pvm, pwm_value_t{0}, pwm_max)};
            return v;
        } 
        
        inline static Ramp make_ramp(const RampPoint& start, const RampPoint& end, uint8_t steps) {
            Ramp ramp;
            ramp.start = convert(start);
            ramp.end = convert(end);
            ramp.steps = steps;
            ramp.delta.tv = (ramp.start.tv - ramp.end.tv) / steps;
            ramp.delta.pwm = (ramp.end.pwm - ramp.start.pwm) / steps;
            return ramp;
        }
        
        inline static Ramp ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
        
        inline static RampValue actualRV = ramp.start;
        
        inline static void init() {
            OL::init();
            Com::init();
            Capture::init();
            Capture::off();
            Capture::noiseCancel();
        }
        inline static void run() {
            spin();
            if ((mState != State::Off)) {
                Capture::template periodic<Capture::flags_type::ocfa>([&](){
                    periodic();    
                });            
            }
        }
        
        inline static void off() {
            Com::off();
            Capture::off();
            mState = State::Off;
        }
        inline static void start() {
            actualRV = ramp.start;
            Capture::reset();
            if (mAlignCounter < alignValueCount) {
                Capture::ocra(std::numeric_limits<timer_value_t>::max());
                ++mAlignCounter;
            }
            else {
                Capture::ocra(alignValueLast);
            }
            //        Timer::ocra(alignValue);
            Capture::template prescale<prescaler>();
            mDelayFactor = 4;
            mState = State::Align;
            mAlignCounter = 0;
            mStep = 0;
        }
        
        inline static uint16_t period = 0;
        
        inline static void enableOffloader() {
            Com::disable();
            OL::enable();
        }
        
        inline static void disableOffloder() {
            OL::disable();
            Com::enable();
        }
        
        inline static void spin() {
            switch(mState) {
            case State::ClosedLoop:
                Capture::template periodic<Capture::flags_type::icf>([&](){ // zero crossing
                    //                db1::on();
                    auto icr = Capture::icr();
                    if (icr >= period / 4) {
                        enableOffloader();                
                        period = icr;
                        uint16_t delay = std::min(period / mDelayFactor, uint16_t{2500}); // todo: Konstante
                        if (delay < 50) { // 500ns * 50 = 25us
                            mState = State::ClosedLoopCommute;
                        }
                        else {
                            Capture::ocra(delay);
                            Capture::template clearFlag<Capture::flags_type::ocfa>();
                            mState = State::ClosedLoopComDelay;
                        }
                        Capture::reset();
                    } 
                    else {
                        Capture::template clearFlag<Capture::flags_type::icf>();
                    }
                    //                db1::off();
                });
                break;
            case State::ClosedLoopCommute:
                //            db2::on();
                Com::next();
                
                Capture::template clearFlag<Capture::flags_type::icf>();
                mState = State::ClosedLoop;
                
                //            db2::off();
                
                OL::run();
                
                disableOffloder();
                
                //            Timer::ocra(4 * period);
                //            Timer::template clearFlag<Timer::flags_type::ocfa>();
                
                break;
            default:
                break;
            }
        }
        inline static void periodic() {
            switch(mState) {
            case State::Align:
                Com::startPosition();
                Capture::reset();
                if (mAlignCounter < alignValueCount) {
                    Capture::ocra(std::numeric_limits<timer_value_t>::max());
                    ++mAlignCounter;
                }
                else {
                    Capture::ocra(alignValueLast);
                    mState = State::RampUp;
                }
                PWM::b(actualRV.pwm.integer());
                Com::on();
                break;
            case State::RampUp:
                Com::next();
                Capture::reset();
                PWM::b(actualRV.pwm.integer());
                if (mStep < ramp_steps) {
                    Capture::ocra(actualRV.tv);
                    ++mStep;
                    actualRV += ramp.delta;
                }
                else {
                    Capture::ocra(4 * actualRV.tv); // timeout
                    mState = State::ClosedLoop;
                    actualRV.pwm = pwmStart;
                    period = actualRV.tv;
                    Capture::template clearFlag<Capture::flags_type::icf>();
                }
                break;
            case State::ClosedLoop:
                break;
            case State::ClosedLoopComDelay:
                PWM::b(actualRV.pwm.integer());
                mState = State::ClosedLoopCommute;
                break;
            case State::ClosedLoopError:
                off();
                break;
            default: 
                assert(false);
                break;
            }
        }
        inline static void delayInc() {
            mDelayFactor += 1;
        }
        inline static void delayDec() {
            mDelayFactor = std::max(4, mDelayFactor - 1);
        }
        inline static void tvInc() {
            ramp.end.tv += 100;
        }
        inline static void tvDec() {
            ramp.end.tv -= 100;
        }
        inline static void pwmInc() {
            if (actualRV.pwm.integer() < 255) {
                actualRV.pwm += etl::FixedPoint<uint16_t, 8>{1.0};
            }
        }
        inline static void pwmDec() {
            if (actualRV.pwm.integer() > 0) {
                actualRV.pwm -= etl::FixedPoint<uint16_t, 8>(1.0);
            }
        }
        inline static void reset() {
            ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
        }
        inline static State mState = State::Off;
        inline static  uint8_t mStep = 0;
        inline static uint8_t mDelayFactor = 4;
        inline static uint8_t mAlignCounter = 0;
    };
}
