#pragma once

#include <etl/staticstring.h>

template<etl::StaticString... SS>
struct InfoTexts : etl::StringList<SS...> {};

template<etl::StaticString... SS>
struct StateTexts : etl::StringList<SS...> {};

template<typename SL, typename ST>
struct Fsm3;

template<etl::StaticString... ST, etl::StaticString... IT>
struct Fsm3<InfoTexts<IT...>, StateTexts<ST...>> {
    template<auto N>
    static inline const char* getInfoText() {
        return &IT...[N].data[0];
    }
    template<auto N>
    static inline const char* getStateText() {
        return &ST...[N].data[0];
    }
};


template<auto N, typename systemTimer>
struct TestFsm {
    enum class State : uint8_t {
        Idle,
        Phase1, Phase2, Phase3,
        Done
    };
    static inline void start() {
        if (mState == State::Idle) {
            mState = State::Phase1;
            mStateTick.reset();
        }
    }    
    static inline void reset() {
        if (mState == State::Done) {
            mState = State::Idle;
            mStateTick.reset();
        }
    }    
    static inline bool status() {
        return !((mState == State::Idle) || (mState == State::Done));
    }    
    static inline uint8_t stateIndex() {
        switch(mState) {
        case State::Phase1:
            return 0;
            break;
        case State::Phase2:
            return 1;
            break;
        case State::Phase3:
            return 2;
            break;
        case State::Done:
            return 3;
            break;
        default:
            return 0;
            break;
        }
    }
    static inline constexpr External::Tick<systemTimer> stateTicks{2000ms};
    static inline void ratePeriodic() {
        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Idle:
            break;
        case State::Phase1:
            mStateTick.on(stateTicks, []{
               mState = State::Phase2; 
            });
            break;
        case State::Phase2:
            mStateTick.on(stateTicks, []{
               mState = State::Phase3; 
            });
            break;
        case State::Phase3:
            mStateTick.on(stateTicks, []{
               mState = State::Done; 
            });
            break;
        case State::Done:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
        }
    }    
private:
    static inline State mState{State::Idle};
    static inline External::Tick<systemTimer> mStateTick;
};
