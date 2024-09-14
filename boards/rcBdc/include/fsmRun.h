#pragma once

template<typename Timer>
struct RunFsm {
    using systemTimer = Timer;

    enum class State : uint8_t {Idle, Stop, Forward, Backward};
    enum class Event : uint8_t {NoEvent, Start};

    static inline void start() {
        if (mState == State::Idle) {
            mLastEvent = Event::Start;
        }
    }
    static inline bool process() {
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Idle:
            if (std::exchange(mLastEvent, Event::NoEvent) == Event::Start) {
                mState = State::Stop;
            }
            break;
        case State::Stop:
            break;
        case State::Forward:
            break;
        case State::Backward:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Idle:
                break;
            case State::Stop:
                break;
            case State::Forward:
                break;
            case State::Backward:
                break;
            }
        }
        return false;
    }
    private:
    static inline Event mLastEvent = Event::NoEvent;
    static inline State mState = State::Idle;
    static inline External::Tick<systemTimer> mStateTick;
};

