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
    
    template<typename HallPinSet, typename Capture, typename PWM, typename Com>
    struct ControllerWithHall {
        typedef typename Capture::value_type timer_value_t;
        typedef typename PWM::value_type pwm_value_t;
        
        static inline constexpr pwm_value_t pwm_max = PWM::max();
//        std::integral_constant<pwm_value_t, pwm_max>::_;
        
        enum class State : uint8_t {Off, Start, StartWait, StartError, ClosedLoop, Commute, CommuteWait, ClosedLoopError};
        
        static inline constexpr uint16_t prescaler = 8; 
        static inline constexpr auto fTimer = Project::Config::fMcu / prescaler;
//        std::integral_constant<uint32_t, fTimer.value>::_;
        
        static inline constexpr auto d1 = 20_ms * fTimer;
//        std::integral_constant<decltype(d1), d1>::_;
        
        static inline constexpr auto pwmStart = etl::FixedPoint<uint16_t, 8>{(double)expand(8_ppc, pwm_value_t{0}, pwm_max)};
//        std::integral_constant<uint16_t, pwmStart.integer()>::_;
        
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
            Com::init();
            Capture::init();
            Capture::off();
            Capture::noiseCancel();
        }
        
        
        inline static void off() {
            Com::off();
            Capture::off();
            mState = State::Off;
            mActualPwm = 0;
        }

        inline static void start() {
            Capture::reset();
            Capture::template prescale<prescaler>();
            Capture::template clearFlag<Capture::flags_type::ocfa>();
            Capture::template clearFlag<Capture::flags_type::tov>();
            Capture::ocra(d1);
            Com::off();
            Com::startPosition();
            mActualPwm = pwmStart.integer();
            mState = State::Start;
        }
        
        inline static void periodic() {
            auto lastState = mState;
            static std::byte lastHalls{0};
            std::byte halls = HallPinSet::read();
            switch(mState) {
            case State::Start:
                PWM::b(mActualPwm);
                Com::set(hallToStep(halls));
                Capture::reset();
                Capture::template clearFlag<Capture::flags_type::tov>();
                mState = State::StartWait;
                break;
            case State::StartWait:
                if (halls != lastHalls) {
                    lastHalls = halls;
                    Com::set(hallToStep(halls));
                    Capture::template clearFlag<Capture::flags_type::ocfa>();
                    mState = State::ClosedLoop;
                }
                else {
                    Capture::template periodic<Capture::flags_type::tov>([&](){
                        mState = State::StartError;                        
                    });
                }
                break;
            case State::StartError:
                Com::off();
                break;
            case State::ClosedLoop:
                if (halls != lastHalls) {
                    
                    mLoopEstimate = (Capture::counter() + 4 * mLoopLaste) / 5;
                    Capture::ocra(mLoopEstimate / 4 + mDelay);
                    Capture::reset();
                    
                    mLoopLaste = mLoopEstimate;
                    lastHalls = halls;
                    mState = State::CommuteWait;
                }
                else {
                    Capture::template periodic<Capture::flags_type::tov>([&](){
                        mState = State::ClosedLoopError;                        
                    });
                }
                break;
            case State::CommuteWait:
                Capture::template periodic<Capture::flags_type::ocfa>([&](){
                    mState = State::Commute;                        
                });
                break;
            case State::Commute:
                Com::set(hallToStep(halls));
                mState = State::ClosedLoop;
                break;
            case State::ClosedLoopError:
                Com::off();
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
            PWM::b(mActualPwm);
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
            PWM::b(mActualPwm);
        }
        inline static void pwmDec() {
            --mActualPwm;
            PWM::b(mActualPwm);
        }

        inline static pwm_value_t mActualPwm = 0;
        inline static State mState = State::Off;
        
        inline static uint16_t mDelay = 10;
        inline static uint16_t mLoopEstimate = 0;
        inline static uint16_t mLoopLaste = 0;
        
    };
}
