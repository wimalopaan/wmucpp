#pragma once

#include <mcu/avr.h>
#include <mcu/pgm/pgmarray.h>

namespace External {
    
    template<typename PPM, typename Timer, typename V, typename Pos>
    struct EscFsm {
        enum class State : uint8_t {Undefined, Init, Off, OffWait, Run};
    
        static constexpr External::Tick<Timer> initTimeoutTicks{100_ms};
        static constexpr External::Tick<Timer> thrTimeoutTicks{5_ms};
        static constexpr External::Tick<Timer> offTimeoutTicks{500_ms};
    
        using value_type = V;
        using ranged_type = value_type::ranged_type;
        
        static inline constexpr uint16_t offOffset{10};
        static inline constexpr ranged_type offValue = ranged_type{ranged_type::Mid};
        static inline constexpr ranged_type offThresh = ranged_type{ranged_type::Mid + offOffset};
        
        inline static void set(const value_type& v) {
            if (v) {
                mTarget = v.toRanged();
            }
        }
        inline static void off() {
            mTarget = offValue;
        }    
            
        inline static void init() {
        }    
        inline static void periodic() {
        }    
        inline static void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTicks;
            switch(mState) {
            case State::Undefined:
                mState = State::Init;
                break;
            case State::Init:
                mStateTicks.on(initTimeoutTicks, []{
                    mState = State::Off;
                });
                break;
            case State::Off:
                if (mTarget > offThresh) {
                    mState = State::Run;
                }
                break;
            case State::OffWait:
                mStateTicks.on(offTimeoutTicks, []{
                    mState = State::Off;
                });
                break;
            case State::Run:
                if ((mTarget <= offThresh) && (mActual <= offThresh)) {
                    mState = State::OffWait;
                }
                (++mThrTicks).on(thrTimeoutTicks, []{
                    if (mTarget > mActual) {
                        mActual += 1;
                    }            
                    if (mTarget < mActual) {
                        mActual -= 1;
                    }            
                    set();            
                });
                break;
            }
            if (oldState != mState) {
                mStateTicks.reset();
                switch(mState) {
                case State::Undefined:
                    break;
                case State::Init:
                    mTarget = mActual = offValue;
                    break;
                case State::Off:
                    mTarget = mActual = offValue;
                    set();
                    break;
                case State::OffWait:
                    break;
                case State::Run:
                    break;
                }
            }
        }    
    private:
        static inline void set() {
            const auto v = AVR::Pgm::scaleTo<typename PPM::ranged_type>(mActual);
            PPM::set(v);
        }
        
        
    //    ranged_type::_;
        static inline ranged_type mTarget;
        static inline ranged_type mActual;
        static inline State mState{State::Undefined};
        static inline External::Tick<Timer> mStateTicks;    
        static inline External::Tick<Timer> mThrTicks;    
    };
    
}
