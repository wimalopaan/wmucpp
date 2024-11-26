#pragma once

#include <cstdint>

#include "tick.h"
#include "etl/fifo.h"

template<typename Out, typename Timer>
struct MessageBuffer {
    using systemTimer = Timer;
    struct Entry {
        using value_type = std::byte;
        static inline constexpr uint8_t size = 64;
        void operator=(const std::span<volatile uint8_t>& s) {
            length = std::min((uint8_t)s.size(), size);
            auto ptr = &s[0];
            for(auto& v: message) {
                v = (value_type)*ptr++;
            }
        }
        void operator=(const std::span<volatile std::byte>& s) {
            length = std::min((uint8_t)s.size(), size);
            auto ptr = &s[0];
            for(auto& v: message) {
                v = *ptr++;
            }
        }
        uint8_t length;
        std::array<value_type, size> message;
    };
    static inline void enqueue(const auto& c) {
        mFifo.emplace_back(c);
    }

    static inline constexpr External::Tick<systemTimer> interMessageTicks{1ms};
    enum class State : uint8_t {Idle, Wait, Send};

    static inline void periodic() {
        switch (mState) {
        case State::Idle:
            if (Out::isIdle() && !mFifo.empty()) {
                mState = State::Wait;
                mStateTick.reset();
            }
            break;
        case State::Wait:
            break;
        case State::Send:
            break;
        }
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        switch (mState) {
        case State::Idle:
            break;
        case State::Wait:
            mStateTick.on(interMessageTicks, []{
                mState = State::Send;
            });
            break;
        case State::Send:
            mState = State::Idle;
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch (mState) {
            case State::Idle:
                break;
            case State::Wait:
                break;
            case State::Send:
                send();
                break;
            }
        }
    }
    private:
    static inline void send() {
        volatile std::byte* const data = Out::outputBuffer();
        const auto& m = mFifo.front();
        for(uint8_t i = 0; (i < m.length) && (i < Out::size); ++i) {
            data[i] = m.message[i];
        }
        Out::startSend(m.length);
        mFifo.pop_front();
    }
    static inline External::Tick<systemTimer> mStateTick{1ms};
    static inline State mState = State::Idle;
    static inline etl::FiFo<Entry> mFifo;
};
