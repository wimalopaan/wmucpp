#pragma once

#include <cstdint>
#include <mcu/internals/systemclock.h>

namespace External {
    template<typename TWI, typename Timer>
    struct DS3231 {
        using twi = TWI;
        using timer = Timer;
        
        static inline constexpr auto address = AVR::Twi::Address{0b1101000};
        static inline constexpr External::Tick<timer> initTicks{100_ms};
        
        static inline void init() {
            twi::init();
        }
        static inline void periodic() {
            twi::periodic();
        }
        
        enum class State : uint8_t {Undefined, Init, Run};

        static inline bool isIdle() {
            return twi::isIdle();
        }
        
        static inline void ratePeriodic() {
            const auto oldState{mState};
            ++mStateTicks;
            
            switch(mState) {
            case State::Undefined:
                mStateTicks.on(initTicks, []{
                    mState = State::Init;
                });
                break;
            case State::Init:
                break;
            case State::Run:
                break;
            }
            if (oldState != mState) {
                mStateTicks.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    twi::write(address, {0x0e, 0x40});                
                    break;
                case State::Run:
                    break;
                }
            }
        }
    private:
        static inline State mState{State::Undefined};
        static inline External::Tick<timer> mStateTicks;
        
    };
}
