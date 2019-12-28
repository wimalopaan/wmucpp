#pragma once

#include <mcu/avr.h>
#include <mcu/common/isr.h>
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
    
    namespace Sensorless {
        
        namespace Experimental2 {
            template<typename RotTimer, typename PWM, typename Com, typename AC>
            struct Controller {
                using all_channels = typename PWM::all_channels;
                
                typedef typename RotTimer::value_type timer_value_t;
                typedef typename PWM::value_type pwm_value_t;
                
                struct RampValue {
                    constexpr RampValue() = default;
                    constexpr RampValue(timer_value_t t, pwm_value_t p) : tv(t), pwm(p) {} 
                    timer_value_t tv = 0;
                    pwm_value_t pwm = 0;
                    
                    inline constexpr RampValue& operator+=(const RampValue& rhs) {
                        tv -= rhs.tv;
                        pwm += rhs.pwm;
                        return *this;
                    }
                };
                
                struct Ramp {
                    constexpr Ramp() = default;
                    RampValue start;
                    RampValue end;
                    RampValue delta;
                    uint8_t   steps = 0;
                };
                
                struct RampPoint {
                    std::chrono::microseconds time;
                    External::Units::percent pvm;
                };
                
                enum class State : uint8_t {Off, Align, Align2, RampUp, ClosedLoop, ClosedLoopError};
                
                static inline constexpr auto fRotTimer = RotTimer::frequency();
                
                // 190KV / 64mm    
                static inline constexpr RampPoint ramp_start = {20000_us,10_ppc};
                static inline constexpr RampPoint ramp_end   = {2000_us, 12_ppc};
                static inline constexpr uint8_t ramp_steps = 50;
                
                
                //    static inline constexpr auto x = expand(5_ppc, pwm_value_t{0}, pwm_max);
                //    std::integral_constant<decltype(x), x>::_;
                
//                static inline auto pwmStart = expand(15_ppc, pwm_value_t{0}, PWM::max());
                
                inline static auto mAlignTime = 200_ms;
                static inline uint16_t alignValue = mAlignTime * fRotTimer;
                
                // 1000KV/37mm
                //    static inline RampPoint ramp_start = {20000_us, 3_ppc};
                //    static inline RampPoint ramp_end   = {2000_us, 6_ppc};
                //    static inline uint8_t ramp_steps = 50;
                //    static inline auto pwmStart = FixedPoint<uint16_t, 8>{(double)std::expand(5_ppc, uint8_t{0}, pwm_max)};
                
                inline static constexpr RampValue convert(const RampPoint& p) {
                    RampValue v;
                    v.tv = p.time * fRotTimer;
                    //                    v.pwm = expand(p.pvm, pwm_value_t{0}, PWM::max());
                    v.pwm = expand(p.pvm, pwm_value_t{0}, pwm_value_t{1000});
                    return v;
                } 
                
                inline static constexpr Ramp make_ramp(const RampPoint& start, const RampPoint& end, uint8_t steps) {
                    Ramp ramp;
                    ramp.start = convert(start);
                    ramp.end = convert(end);
                    ramp.steps = steps;
                    ramp.delta.tv = (ramp.start.tv - ramp.end.tv) / steps;
                    ramp.delta.pwm = std::min((ramp.end.pwm - ramp.start.pwm) / steps, 1u);
                    return ramp;
                }
                
                inline static constexpr Ramp ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
                
                //                inline static const Ramp ramp{{}};
                
//                                std::integral_constant<decltype(ramp.start.pwm), ramp.start.pwm>::_;
//                                std::integral_constant<decltype(ramp.start.tv), ramp.start.tv>::_;
//                                std::integral_constant<decltype(ramp.end.pwm), ramp.end.pwm>::_;
//                                std::integral_constant<decltype(ramp.end.tv), ramp.end.tv>::_;
//                                std::integral_constant<decltype(ramp.delta.pwm), ramp.delta.pwm>::_;
//                                std::integral_constant<decltype(ramp.delta.tv), ramp.delta.tv>::_;
                
                inline static RampValue actualRV = ramp.start;
                
                inline static void init() {
                    AC::init();
                    AC::template enableInterrupts<false>();
                    Com::init();
                    RotTimer::init();
                    off();
                }
                inline static void off() {
                    AC::template enableInterrupts<false>();
                    Com::off();    
                    mState = State::Off;
                    actualRV = ramp.start;
                }
                
                inline static void start() {
                    actualRV = ramp.start;
                    mState = State::Align;    
                    mStep = 0;
                }
                
                inline static void periodic() {
                    switch(mState) {
                    case State::Off:
                        break;
                    case State::Align:
                        Com::startPosition();
                        Com::on_full_state();
                        PWM::template duty<all_channels >(actualRV.pwm);
                        RotTimer::period(alignValue);
                        RotTimer::reset();
                        mState = State::Align2;
                        break;
                    case State::Align2:
                        RotTimer::onPeriodic([&](){
                            RotTimer::period(actualRV.tv);
                            RotTimer::reset();
                            mState = State::RampUp;
                            AC::template enableInterrupts<true>();
                            mCommutes = 0;
                        });
                        break;
                    case State::RampUp:
                        RotTimer::onPeriodic([&](){
                            if (mStep < ramp_steps) {
                                Com::next();
                                ++mStep;
                                actualRV += ramp.delta;
                                PWM::template duty<all_channels >(actualRV.pwm);
                                RotTimer::period(actualRV.tv);
                                RotTimer::reset();
                            }
                        });
//                        if ((mStep >= ramp_steps)) {
                        if ((mStep >= ramp_steps) || ((mCommutes > mGoodCommutes) && (mStep >= (ramp_steps / 2)))) {
                            actualRV.pwm = ramp.end.pwm;
                            PWM::template duty<all_channels>(actualRV.pwm);
                            mState = State::ClosedLoop;
                            RotTimer::period(std::numeric_limits<uint16_t>::max());
                            RotTimer::reset();
                            mCommutationDiff = 0;
                            mCommutes = 0;
                            mErrorCount = 0;
                        }
                        break;
                    case State::ClosedLoop:
                        break;
                    case State::ClosedLoopError:
                        off();
                        break;
                    default: 
                        assert(false);
                        break;
                    }
                }
                
                inline static void check() { // every 2ms
                    switch(mState) {
                    case State::ClosedLoop:
                    {
                        static uint16_t cycles = 0;
                        ++cycles;
                        
                        static uint16_t lastCommutes = 0;
                        uint16_t d;
                        {
                            etl::Scoped<etl::DisbaleInterrupt<>> di;
                            d = mCommutes;
                        }
                        if (cycles == 1000) { // 100ms
                            if (d == lastCommutes) {
                                ++mErrorCount;
                            }
                            cycles = 0;
                        }
                        lastCommutes = d;
                        {
                            etl::Scoped<etl::DisbaleInterrupt<>> di;
                            d = mCommutationDiff;
                        }
                        if (d > 16000) { // 500ms
                            ++mErrorCount;
                        }
                        if (d <= 2) { 
                            ++mErrorCount;
                        }
                        if (mErrorCount > 10) {
//                            mState = State::ClosedLoopError;
                        }
                    }
                        break;
                    default:
                        break;
                    }
                }
                
                struct AcHandler : AVR::IsrBaseHandler<AVR::ISR::AdComparator<0>::Edge>{
                    static inline void isr() {
                        AC::onInterrupt([&](){
                            static uint16_t last = 0;
//                            if (mState == State::ClosedLoop) {
//                                Com::next();
//                            }
//                            return;

                            uint16_t actual = RotTimer::counter();
                            if (actual >= last) {
                                mCommutationDiff = actual - last;
                            }
                            else {
                                mCommutationDiff = actual + (std::numeric_limits<uint16_t>::max() - last); 
                            }
                            last = actual;
                            
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
                            mCommutes = mCommutes + 1;
                            if (mState == State::ClosedLoop) {
                                Com::next();
                            }
                        });
                    };
                };
                
                inline static void delayInc() {
                }
                
                inline static void delayDec() {
                }

                inline static void tvInc() {
                    ramp.end.tv += 100;
                }
                inline static void tvDec() {
                    ramp.end.tv -= 100;
                }
                inline static void pwmInc() {
                    actualRV.pwm += 1;
                    PWM::template duty<all_channels>(actualRV.pwm);
                }
                inline static void pwmfast() {
                    actualRV.pwm += 100;
                    PWM::template duty<all_channels>(actualRV.pwm);
                }
                inline static void pwmDec() {
                    actualRV.pwm -= 1;
                    PWM::template duty<all_channels>(actualRV.pwm);
                }
                inline static void reset() {
                    ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
                }
                
                inline static State mState = State::Off;
                inline static  uint8_t mStep = 0;
                inline static uint8_t mDelayFactor = 4;
                
                inline static uint16_t mErrorCount = 0;
               
                inline static uint8_t mGoodCommutes = 10;
                
                inline static volatile uint16_t mCommutationDiff = 0;
                inline static volatile uint16_t mCommutes = 0;
                
                inline static volatile uint16_t mDelay = 4;
                inline static volatile uint16_t mMaxDelay = 10;
                
            };
        } //! namespace
        
        namespace Experimental {
            template<typename RotTimer, typename ComTimer, typename PWM, typename Com, typename AC>
            struct Controller {
                using all_channels = typename PWM::all_channels;
                
                typedef typename RotTimer::value_type timer_value_t;
                typedef typename PWM::value_type pwm_value_t;
                
                //                static inline constexpr pwm_value_t pwm_max = PWM::max();
                
                struct RampValue {
                    constexpr RampValue() = default;
                    constexpr RampValue(timer_value_t t, pwm_value_t p) : tv(t), pwm(p) {} 
                    timer_value_t tv = 0;
                    pwm_value_t pwm = 0;
                    
                    inline constexpr RampValue& operator+=(const RampValue& rhs) {
                        tv -= rhs.tv;
                        pwm += rhs.pwm;
                        return *this;
                    }
                };
                
                struct Ramp {
                    constexpr Ramp() = default;
                    RampValue start;
                    RampValue end;
                    RampValue delta;
                    uint8_t   steps = 0;
                };
                
                struct RampPoint {
                    std::chrono::microseconds time;
                    External::Units::percent pvm;
                };
                
                enum class State : uint8_t {Off, Align, Align2, RampUp, ClosedLoop, ClosedLoopComDelay, ClosedLoopCommute, ClosedLoopError};
                
                static inline constexpr auto fComTimer = ComTimer::frequency();
                static inline constexpr auto fRotTimer = RotTimer::frequency();
                
                // 190KV / 64mm    
                static inline constexpr RampPoint ramp_start = {20000_us, 5_ppc};
                static inline constexpr RampPoint ramp_end   = {10000_us, 10_ppc};
                static inline constexpr uint8_t ramp_steps = 50;
                
                //    static inline constexpr auto x = expand(5_ppc, pwm_value_t{0}, pwm_max);
                //    std::integral_constant<decltype(x), x>::_;
                
                static inline auto pwmStart = expand(10_ppc, pwm_value_t{0}, PWM::max());
                
                static inline constexpr uint16_t alignValue2 = 200_ms * fRotTimer;
                
                // 1000KV/37mm
                //    static inline RampPoint ramp_start = {20000_us, 3_ppc};
                //    static inline RampPoint ramp_end   = {2000_us, 6_ppc};
                //    static inline uint8_t ramp_steps = 50;
                //    static inline auto pwmStart = FixedPoint<uint16_t, 8>{(double)std::expand(5_ppc, uint8_t{0}, pwm_max)};
                
                inline static constexpr RampValue convert(const RampPoint& p) {
                    RampValue v;
                    v.tv = p.time * fRotTimer;
                    //                    v.pwm = expand(p.pvm, pwm_value_t{0}, PWM::max());
                    v.pwm = expand(p.pvm, pwm_value_t{0}, pwm_value_t{1000});
                    return v;
                } 
                
                inline static constexpr Ramp make_ramp(const RampPoint& start, const RampPoint& end, uint8_t steps) {
                    Ramp ramp;
                    ramp.start = convert(start);
                    ramp.end = convert(end);
                    ramp.steps = steps;
                    ramp.delta.tv = (ramp.start.tv - ramp.end.tv) / steps;
                    ramp.delta.pwm = (ramp.end.pwm - ramp.start.pwm) / steps;
                    return ramp;
                }
                
                inline static constexpr Ramp ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
                //                inline static const Ramp ramp{{}};
                
                //                std::integral_constant<decltype(ramp.start.pwm), ramp.start.pwm>::_;
                //                std::integral_constant<decltype(ramp.start.tv), ramp.start.tv>::_;
                //                std::integral_constant<decltype(ramp.end.pwm), ramp.end.pwm>::_;
                //                std::integral_constant<decltype(ramp.end.tv), ramp.end.tv>::_;
                //                std::integral_constant<decltype(ramp.delta.pwm), ramp.delta.pwm>::_;
                //                std::integral_constant<decltype(ramp.delta.tv), ramp.delta.tv>::_;
                
                inline static RampValue actualRV = ramp.start;
                
                inline static void init() {
                    Com::init();
                    ComTimer::init();
                    RotTimer::init();
                    off();
                }
                inline static void off() {
                    Com::off();    
                    ComTimer::off();
                    mState = State::Off;
                    actualRV = ramp.start;
                }
                
                inline static void start() {
                    ComTimer::on();
                    actualRV = ramp.start;
                    mState = State::Align;    
                    mStep = 0;
                }
                
                inline static void periodic() {
                    switch(mState) {
                    case State::Off:
                        break;
                    case State::Align:
                        Com::startPosition();
                        PWM::template duty<all_channels >(actualRV.pwm);
                        RotTimer::period(alignValue2);
                        RotTimer::reset();
                        mState = State::Align2;
                        break;
                    case State::Align2:
                        RotTimer::onPeriodic([&](){
                            RotTimer::period(actualRV.tv);
                            RotTimer::reset();
                            mState = State::RampUp;
                        });
                        break;
                    case State::RampUp:
                        RotTimer::onPeriodic([&](){
                            if (mStep < ramp_steps) {
                                Com::next();
                                ++mStep;
                                actualRV += ramp.delta;
                                PWM::template duty<all_channels >(actualRV.pwm);
                                RotTimer::period(actualRV.tv);
                                RotTimer::reset();
                            }
                            else {
                                //                                mState = State::ClosedLoop;
                            }
                        });
                        if (mStep >= ramp_steps) {
                            mState = State::ClosedLoop;
                        }
                        break;
                    case State::ClosedLoop:
                        AC::onEdge([&](){
                            Com::next();
                            //                            ComTimer::period(5000);
                            //                            mState = State::ClosedLoopComDelay;
                        });
                        break;
                    case State::ClosedLoopComDelay:
                        ComTimer::periodic([&](){
                            mState = State::ClosedLoopCommute;
                        });
                        break;
                    case State::ClosedLoopCommute:
                        Com::next();
                        mState = State::ClosedLoop;
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
                    actualRV.pwm += 1;
                    PWM::template duty<all_channels >(actualRV.pwm);
                }
                inline static void pwmDec() {
                    actualRV.pwm -= 1;
                    PWM::template duty<all_channels >(actualRV.pwm);
                }
                inline static void reset() {
                    ramp = make_ramp(ramp_start, ramp_end, ramp_steps);
                }
                inline static State mState = State::Off;
                inline static  uint8_t mStep = 0;
                inline static uint8_t mDelayFactor = 4;
            };
            
        } //! namespace
        
        
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
    
}
