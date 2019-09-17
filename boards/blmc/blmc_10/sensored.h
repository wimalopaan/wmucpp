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
    
    namespace Sensored {

        namespace Experimental2 {

            template<typename HallPinSet, typename RotTimer, typename ComTimer, typename PWM, typename Commuter, typename DBG = void>
            struct Controller {
                typedef typename RotTimer::value_type timer_value_t;
                typedef typename PWM::value_type pwm_value_t;
                
                enum class State : uint8_t {Off, Start, StartWait, StartError, ClosedLoop, Commute, CommuteWait, ClosedLoopError};
                
                static inline constexpr auto fRotTimer = RotTimer::frequency();
//                        std::integral_constant<uint32_t, fRotTimer.value>::_;
                
                static inline constexpr auto d1 = 900_ms * fRotTimer;
                //        static inline constexpr auto d2 = 300_ms * fTimer;
                //        std::integral_constant<decltype(d1), d1>::_;

                static inline constexpr auto fComTimer = ComTimer::frequency();
                //        std::integral_constant<decltype(d1), d1>::_;
                
                static inline constexpr auto fRatio = fComTimer / fRotTimer;
//                std::integral_constant<decltype(fRatio), fRatio>::_;
                
                static inline constexpr uint8_t comDelayPart = 4;
                
                static_assert(fRatio >= comDelayPart);
                
                static inline constexpr uint16_t rotTimerToComTimer = fRatio / comDelayPart;
//                std::integral_constant<decltype(rotTimerToComTimer), rotTimerToComTimer>::_;

                static inline constexpr uint16_t rotTimerThreash = std::numeric_limits<uint16_t>::max() / rotTimerToComTimer;
//                std::integral_constant<decltype(rotTimerThreash), rotTimerThreash>::_;
                
                static inline constexpr uint8_t exponential = 16;
                
                struct HallHandler : IsrBaseHandler<AVR::ISR::Port<typename HallPinSet::port::name_type>> {
                    inline static void isr() {
                        HallPinSet::onInterrupt([&](){
                            if constexpr(!std::is_same_v<DBG, void>) {
                                DBG::toggle();
                            }
                            if (mState == State::ClosedLoop) {
                                std::byte halls = HallPinSet::read();
                                Commuter::set(hallToStep(halls));
                            }
                        });    
                    }
                };
                
                static inline constexpr std::array<std::byte, 6> hallValues {
                    0b00000001_B,
                    0b00000101_B,
                    0b00000100_B,
                    0b00000110_B,
                    0b00000010_B,
                    0b00000011_B,
                };        
                
                static inline constexpr auto hallValuesStepLut = []{
                    std::array<etl::uint_ranged_circular<uint8_t, 0, 5>, 8> values {};
                    for(uint8_t s = 0; const auto& hv: hallValues) {
                        values[std::to_integer(hv)] = s++;
                    }    
                    return values;
                }();
                
                static inline constexpr auto hallToStep(std::byte h) {
                    uint8_t i = std::to_integer(h & 0x07_B);
                    return hallValuesStepLut[i];
                }
                
                inline static void init() {
                    HallPinSet::template dir<Input>();
                    HallPinSet::template attributes<Meta::List<AVR::Attributes::Interrupt<AVR::Attributes::BothEdges>>>();
                    HallPinSet::pullup();
                    Commuter::init();
                    RotTimer::init();
                    ComTimer::init();
                    mState = State::Off;
                    mActualPwm = 0;
                    Commuter::off();
                }
                
                inline static void off() {
                    Commuter::off();
                    mState = State::Off;
                    mActualPwm = 0;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(0);
                }
                
                inline static void start() {
                    mActualPwm = expand(2_ppc, pwm_value_t{0}, PWM::max()); // schwarz
//                    mActualPwm = expand(5_ppc, pwm_value_t{0}, PWM::max());
                    mState = State::Start;
                    RotTimer::period(d1);
                }
                
                inline static void periodic() {
                    auto lastState = mState;
                    static std::byte lastHalls{0};
                    std::byte halls = HallPinSet::read();

                    switch(mState) {
                    case State::Start:
                        lastHalls = halls;
                        PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                        Commuter::set(hallToStep(halls));
                        RotTimer::reset();
                        mState = State::ClosedLoop;
                        break;
                    case State::StartWait:
//                        if (halls != lastHalls) {
//                            lastHalls = halls;
//                            Commuter::set(hallToStep(halls));
//                            RotTimer::reset();
//                            mState = State::ClosedLoop;
//                        }
//                        else {
//                            RotTimer::template periodic<RotTimer::intflags_t::ovf>([&](){
//                                mState = State::StartError;                        
//                            });
//                        }
                        break;
                    case State::StartError:
                        Commuter::off();
                        break;
                    case State::ClosedLoop:
//                        if (halls != lastHalls) {
//                            mLoopEstimate = (RotTimer::counter() + (exponential - 1) * mLoopLast) / exponential;
//                            RotTimer::reset();
//                            if (mLoopEstimate < rotTimerThreash) {
//                                mComPeriod = mLoopEstimate * rotTimerToComTimer;
//                            }
//                            else {
//                                mComPeriod = std::numeric_limits<uint16_t>::max();
//                            }
//                            ComTimer::period(mComPeriod);
//                            mLoopLast = mLoopEstimate;
//                            lastHalls = halls;
//                            mState = State::CommuteWait;
//                        }
//                        else {
//                            RotTimer::template periodic<RotTimer::intflags_t::ovf>([&](){
//                                mState = State::ClosedLoopError;                        
//                            });
//                        }
                        break;
                    case State::CommuteWait:
//                        ComTimer::template periodic([&](){
//                            mState = State::Commute;                        
//                        });
                        break;
                    case State::Commute:
//                        Commuter::set(hallToStep(halls));
//                        mState = State::ClosedLoop;
                        break;
                    case State::ClosedLoopError:
                        Commuter::off();
                        break;
                    default: 
                        break;
                    }
                }
                inline static void pwmInc() {
                    ++mActualPwm;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                }
                inline static void pwmfast() {
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                }
                inline static void pwmDec() {
                    --mActualPwm;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                }
                
                inline static pwm_value_t mActualPwm = 0;
                inline static State mState = State::Off;
                
                inline static uint16_t mLoopEstimate = 0;
                inline static uint16_t mLoopLast = 0;
                inline static uint16_t mComPeriod = 0;
                
            };
            
        }
        
        namespace Experimental {

            template<typename HallPinSet, typename RotTimer, typename ComTimer, typename PWM, typename Commuter>
            struct Controller {
                typedef typename RotTimer::value_type timer_value_t;
                typedef typename PWM::value_type pwm_value_t;
                
                enum class State : uint8_t {Off, Start, StartWait, StartError, ClosedLoop, Commute, CommuteWait, ClosedLoopError};
                
                static inline constexpr auto fRotTimer = RotTimer::frequency();
//                        std::integral_constant<uint32_t, fRotTimer.value>::_;
                
                static inline constexpr auto d1 = 900_ms * fRotTimer;
                //        static inline constexpr auto d2 = 300_ms * fTimer;
                //        std::integral_constant<decltype(d1), d1>::_;

                static inline constexpr auto fComTimer = ComTimer::frequency();
                //        std::integral_constant<decltype(d1), d1>::_;
                
                static inline constexpr auto fRatio = fComTimer / fRotTimer;
//                std::integral_constant<decltype(fRatio), fRatio>::_;
                
                static inline constexpr uint8_t comDelayPart = 4;
                
                static_assert(fRatio >= comDelayPart);
                
                static inline constexpr uint16_t rotTimerToComTimer = fRatio / comDelayPart;
//                std::integral_constant<decltype(rotTimerToComTimer), rotTimerToComTimer>::_;

                static inline constexpr uint16_t rotTimerThreash = std::numeric_limits<uint16_t>::max() / rotTimerToComTimer;
//                std::integral_constant<decltype(rotTimerThreash), rotTimerThreash>::_;
                
                static inline constexpr uint8_t exponential = 16;
                
                static inline constexpr std::array<std::byte, 6> hallValues {
                    0b00000001_B,
                    0b00000101_B,
                    0b00000100_B,
                    0b00000110_B,
                    0b00000010_B,
                    0b00000011_B,
                };        
                
                static inline constexpr auto hallValuesStepLut = []{
                    std::array<etl::uint_ranged_circular<uint8_t, 0, 5>, 8> values {};
                    for(uint8_t s = 0; const auto& hv: hallValues) {
                        values[std::to_integer(hv)] = s++;
                    }    
                    return values;
                }();
                
                static inline constexpr auto hallToStep(std::byte h) {
                    uint8_t i = std::to_integer(h & 0x07_B);
                    return hallValuesStepLut[i];
                }
                
                inline static void init() {
                    HallPinSet::template dir<Input>();
                    HallPinSet::pullup();
                    Commuter::init();
                    RotTimer::init();
                    ComTimer::init();
                    mState = State::Off;
                    mActualPwm = 0;
                    Commuter::off();
                }
                
                inline static void off() {
                    Commuter::off();
                    mState = State::Off;
                    mActualPwm = 0;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(0);
                }
                
                inline static void start() {
                    mActualPwm = expand(10_ppc, pwm_value_t{0}, PWM::max());
                    mState = State::Start;
                    RotTimer::period(d1);
                }
                
                inline static void periodic() {
                    auto lastState = mState;
                    static std::byte lastHalls{0};
                    std::byte halls = HallPinSet::read();
                    switch(mState) {
                    case State::Start:
                        lastHalls = halls;
                        PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                        Commuter::set(hallToStep(halls));
                        RotTimer::reset();
                        mState = State::StartWait;
                        break;
                    case State::StartWait:
                        if (halls != lastHalls) {
                            lastHalls = halls;
                            Commuter::set(hallToStep(halls));
                            RotTimer::reset();
                            mState = State::ClosedLoop;
                        }
                        else {
                            RotTimer::template periodic<RotTimer::intflags_t::ovf>([&](){
                                mState = State::StartError;                        
                            });
                        }
                        break;
                    case State::StartError:
                        Commuter::off();
                        break;
                    case State::ClosedLoop:
                        if (halls != lastHalls) {
                            mLoopEstimate = (RotTimer::counter() + (exponential - 1) * mLoopLast) / exponential;
                            RotTimer::reset();
                            if (mLoopEstimate < rotTimerThreash) {
                                mComPeriod = mLoopEstimate * rotTimerToComTimer;
                            }
                            else {
                                mComPeriod = std::numeric_limits<uint16_t>::max();
                            }
                            ComTimer::period(mComPeriod);
                            mLoopLast = mLoopEstimate;
                            lastHalls = halls;
                            mState = State::CommuteWait;
                        }
                        else {
                            RotTimer::template periodic<RotTimer::intflags_t::ovf>([&](){
                                mState = State::ClosedLoopError;                        
                            });
                        }
                        break;
                    case State::CommuteWait:
                        ComTimer::template periodic([&](){
                            mState = State::Commute;                        
                        });
                        break;
                    case State::Commute:
                        Commuter::set(hallToStep(halls));
                        mState = State::ClosedLoop;
                        break;
                    case State::ClosedLoopError:
                        Commuter::off();
                        break;
                    default: 
                        assert(false);
                        break;
                    }
                    if (lastState != mState) {
                        
                    }
                }
                inline static void pwmInc() {
                    ++mActualPwm;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                }
                inline static void pwmfast() {
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    ++mActualPwm;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                }
                inline static void pwmDec() {
                    --mActualPwm;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                }
                
                inline static pwm_value_t mActualPwm = 0;
                inline static State mState = State::Off;
                
                inline static uint16_t mLoopEstimate = 0;
                inline static uint16_t mLoopLast = 0;
                inline static uint16_t mComPeriod = 0;
                
            };
            
        }
        
        

        template<typename HallPinSet, typename RotTimer, typename PWM, typename Commuter>
        struct Controller {
            typedef typename RotTimer::value_type timer_value_t;
            typedef typename PWM::value_type pwm_value_t;
            
            enum class State : uint8_t {Off, Start, StartWait, StartError, ClosedLoop, Commute, CommuteWait, ClosedLoopError};
            
            static inline constexpr auto fTimer = Project::Config::fRtc;
            //        std::integral_constant<uint32_t, fTimer.value>::_;
            
            static inline constexpr auto d1 = 900_ms * fTimer;
            //        static inline constexpr auto d2 = 300_ms * fTimer;
            //        std::integral_constant<decltype(d1), d1>::_;
            
            static inline constexpr std::array<std::byte, 6> hallValues {
                0b00000001_B,
                0b00000101_B,
                0b00000100_B,
                0b00000110_B,
                0b00000010_B,
                0b00000011_B,
            };        
            
            static inline constexpr auto hallValuesStepLut = []{
                std::array<etl::uint_ranged_circular<uint8_t, 0, 5>, 8> values {};
                for(uint8_t s = 0; const auto& hv: hallValues) {
                    values[std::to_integer(hv)] = s++;
                }    
                return values;
            }();
            
            static inline constexpr auto hallToStep(std::byte h) {
                uint8_t i = std::to_integer(h & 0x07_B);
                return hallValuesStepLut[i];
            }
            
            inline static void init() {
                HallPinSet::template dir<Input>();
                HallPinSet::pullup();
                Commuter::init();
                RotTimer::init();
                mState = State::Off;
                mActualPwm = 0;
                Commuter::off();
            }
            
            inline static void off() {
                Commuter::off();
                mState = State::Off;
                mActualPwm = 0;
                PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(0);
            }
            
            inline static void start() {
                mActualPwm = expand(10_ppc, pwm_value_t{0}, PWM::max());
                mState = State::Start;
                RotTimer::period(d1);
            }
            
            inline static void periodic() {
                auto lastState = mState;
                static std::byte lastHalls{0};
                std::byte halls = HallPinSet::read();
                switch(mState) {
                case State::Start:
                    lastHalls = halls;
                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
                    Commuter::set(hallToStep(halls));
                    RotTimer::reset();
                    mState = State::StartWait;
                    break;
                case State::StartWait:
                    if (halls != lastHalls) {
                        lastHalls = halls;
                        Commuter::set(hallToStep(halls));
                        RotTimer::reset();
                        mState = State::ClosedLoop;
                    }
                    else {
                        RotTimer::template periodic<RotTimer::intflags_t::ovf>([&](){
                            mState = State::StartError;                        
                        });
                    }
                    break;
                case State::StartError:
                    Commuter::off();
                    break;
                case State::ClosedLoop:
                    if (halls != lastHalls) {
                        mLoopEstimate = (RotTimer::counter() + 4 * mLoopLast) / 5;
                        RotTimer::reset();
                        RotTimer::duty(mLoopEstimate / 4 + mDelay);
                        
                        mLoopLast = mLoopEstimate;
                        lastHalls = halls;
                        mState = State::CommuteWait;
                    }
                    else {
                        RotTimer::template periodic<RotTimer::intflags_t::ovf>([&](){
                            mState = State::ClosedLoopError;                        
                        });
                    }
                    break;
                case State::CommuteWait:
                    RotTimer::template periodic<RotTimer::intflags_t::cmp>([&](){
                        mState = State::Commute;                        
                    });
                    mState = State::Commute;                        
                    break;
                case State::Commute:
                    Commuter::set(hallToStep(halls));
                    mState = State::ClosedLoop;
                    break;
                case State::ClosedLoopError:
                    Commuter::off();
                    break;
                default: 
                    assert(false);
                    break;
                }
                if (lastState != mState) {
                    
                }
            }
            inline static void pwmInc() {
                ++mActualPwm;
                PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
            }
            inline static void pwmfast() {
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                ++mActualPwm;
                PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
            }
            inline static void pwmDec() {
                --mActualPwm;
                PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(mActualPwm);
            }
            
            inline static pwm_value_t mActualPwm = 0;
            inline static State mState = State::Off;
            
            inline static uint16_t mDelay = 1;
            inline static uint16_t mLoopEstimate = 0;
            inline static uint16_t mLoopLast = 0;
            
        };
    }
}
