/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <cstddef>
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>
#include <cstring>
#include <type_traits>

#include "etl/algorithm.h"
#include "etl/fixedvector.h"
#include "etl/event.h"
#include "debug_pin.h"

#include "tick.h"

namespace Util {
    using namespace std::literals::chrono_literals;

    template<typename Out, typename ValueType, typename Timer, typename tp, auto Size = 64, auto FifoSize = 32>
    struct MessageBuffer {
        using systemTimer = Timer;
        using value_type = ValueType;
        struct Entry {
            using value_type = ValueType;
            static inline constexpr uint8_t size = Size;
            void operator=(const std::span<const volatile value_type>& s) {
                length = std::min((uint8_t)s.size(), size);
                for(uint8_t i = 0; i < length; ++i) {
                    message[i] = s[i];
                }
            }
            uint8_t length;
            std::array<value_type, size> message;
        };
        static inline void enqueue(const auto& c) {
            mFifo.emplace_back(c);
        }
        template<typename M>
        class Transaction {
            friend M;
            explicit Transaction(Entry& d, const uint8_t type) : mEntry{d} {
                mEntry.message[0] = 0xc8;
                mEntry.message[2] = type;
                mEntry.length = 3;
            }
            inline ~Transaction() {
                const uint8_t length = mEntry.length + 1; // incl. CRC
                mEntry.message[1] = length - 2; // without: startbyte and length, including CRC
                CRC8 csum;
                for(int8_t i = 0; i < length - 2 - 1; ++i) { // without CRC
                    csum += mEntry.message[i + 2];
                }
                push_back((uint8_t)csum);

            }
            Entry& mEntry;
            public:
            using value_type = uint8_t;
            inline void push_back(const uint8_t v) {
                mEntry.message[mEntry.length++] = v;
            }
            inline void push_back(const std::byte v) {
                push_back((uint8_t)v);
            }
            template<typename El, auto L>
            requires((L <= 60) && (sizeof(El) <= 2))
            inline void push_back(const std::array<El, L>& c) {
                for(const El& e: c) {
                    etl::serializeBE(e, *this);
                }
            }
            inline void push_back(const std::pair<uint8_t, uint8_t> v) {
                push_back(v.first);
                push_back(v.second);
            }
            inline void push_back(const uint16_t v) {
                etl::serializeBE(v, *this);
            }
            inline void push_back(const int16_t v) {
                etl::serializeBE(v, *this);
            }
            inline void push_back(const uint32_t v) {
                etl::serializeBE(v, *this);
            }
            inline void push_back(const int32_t v) {
                etl::serializeBE(v, *this);
            }
        };
        static inline void create_back(const uint8_t type, const auto f) {
            mFifo.create_back([&](Entry& d) {
                Transaction<MessageBuffer> w(d, type);
                f(w);
            });
        }

        static inline constexpr External::Tick<systemTimer> interMessageTicks{1ms};

        enum class State : uint8_t {Idle, Wait, Send};
        enum class Event : uint8_t {None, TransmitComplete};

        enum class ResponseEvent : uint8_t {None, Send};

        static inline void freeRun(const bool on) {
            mFreeRun = on;
        }
        static inline void sendNext() {
            mResponseEvent = ResponseEvent::Send;
        }
        static inline void clearEvent() {
            mResponseEvent = ResponseEvent::None;
        }

        static inline void event(const Event e) {
            mEvent = e;
        }
        static inline void periodic() {
            switch (mState) {
            case State::Idle:
                if (!mFifo.empty()) {
                    if (mFreeRun) {
                        send();
                        mState = State::Send;
                    }
                    else {
                        mResponseEvent.on(ResponseEvent::Send, []{
                            send();
                            mState = State::Send;
                        });
                    }
                }
                break;
            case State::Send:
                if (mEvent.is(Event::TransmitComplete)) {
                    mStateTick.reset();
                    mState = State::Wait;
                }
                break;
            case State::Wait:
                break;
            }
        }
        static inline void ratePeriodic() {
            ++mStateTick;
            switch (mState) {
            case State::Idle:
                break;
            case State::Send:
                break;
            case State::Wait:
                mStateTick.on(interMessageTicks, []{
                    mState = State::Idle;
                    mResponseEvent = ResponseEvent::None;
                });
                break;
            }
        }
        private:
        static inline void send() {
            if (mFifo.size() > 0) {
                Out::fillSendBuffer([&](auto& data){
                    const auto& m = mFifo.front();
                    static_assert(m.message.size() == data.size());
                    for(uint8_t i = 0; i < m.length; ++i) {
                        data[i] = m.message[i];
                    }
                    mFifo.pop_front();
                    return m.length;
                });
            }
        }
        static inline bool mFreeRun = true;
        static inline etl::Event<Event> mEvent;
        static inline etl::Event<ResponseEvent> mResponseEvent;
        static inline External::Tick<systemTimer> mStateTick{1ms};
        static inline State mState = State::Idle;
        static inline etl::FiFo<Entry, FifoSize> mFifo;
    };
}
