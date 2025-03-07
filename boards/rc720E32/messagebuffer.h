/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>

#include "tick.h"
#include "etl/fifo.h"

template<typename Out, typename Timer, auto Size = 64>
struct MessageBuffer {
    using systemTimer = Timer;
    struct Entry {
        using value_type = std::byte;
        static inline constexpr uint8_t size = Size;
        void operator=(const std::span<volatile uint8_t>& s) {
            length = std::min((uint8_t)s.size(), size);
            auto ptr = &s[0];
            for(auto& v: message) {
                v = (value_type)*ptr++;
            }
        }
        void operator=(const std::span<volatile std::byte>& s) {
            const std::span<volatile uint8_t> sb{(volatile uint8_t*)s.data(), s.size_bytes()};
            this->operator=(sb);
            // length = std::min((uint8_t)s.size(), size);
            // auto ptr = &s[0];
            // for(auto& v: message) {
            //     v = *ptr++;
            // }
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
