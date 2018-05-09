/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

struct ClockStateMachine final {
    
    enum class State : uint8_t {PreStart, Start, Sync1, Sync2, Sync3, Clock, Error};
    enum class Event : uint8_t {Reset, Start, DCFSync, DCFDecode, DCFError, ReSync};
    
    // todo: für ReSync erweitern: falls Resync nicht erfolgreich -> Zeit beibehalten und ReSyncCounter irgendwie anzeigen (Farbe ander 12).
    template<typename StateEntryCallback = void, typename StateExitCallback = void, typename ProcessCallback = void>
    struct Machine final {
        Machine() = delete;
    private:
        inline static State mState = State::PreStart; // c++17
    public:
        static State state() {
            return mState;
        }
        static void process(Event event) {
            State newState = mState;
            switch (mState) {
            case State::PreStart:
                if (event == Event::Start) {
                    newState = State::Start;
                }
                else if (event == Event::ReSync) {
                    newState = State::Start;
                }
                break;
            case State::Start:
                if (event == Event::DCFSync) {
                    newState = State::Sync1;
                }
                else if (event == Event::Reset) {
                    newState = State::PreStart;
                }
                break;
            case State::Sync1:
                if (event == Event::DCFDecode) {
                    newState = State::Sync2;
                }
                else if (event == Event::Reset) {
                    newState = State::PreStart;
                }
                else if (event == Event::DCFError) {
                    newState = State::Error;
                }
                break;
            case State::Sync2:
                if (event == Event::DCFDecode) {
                    newState = State::Sync3;
                }
                else if (event == Event::Reset) {
                    newState = State::PreStart;
                }
                else if (event == Event::DCFError) {
                    newState = State::Error;
                }
                break;
            case State::Sync3:
                if (event == Event::DCFDecode) {
                    newState = State::Clock;
                }
                else if (event == Event::Reset) {
                    newState = State::PreStart;
                }
                else if (event == Event::DCFError) {
                    newState = State::Error;
                }
                break;
            case State::Clock:
                if (event == Event::ReSync) {
                    newState = State::Start;
                }
                else if (event == Event::Reset) {
                    newState = State::PreStart;
                }
                break;
            case State::Error:
                if (event == Event::DCFSync) {
                    newState = State::Start;
                }
                else if (event == Event::Reset) {
                    newState = State::PreStart;
                }
                break;
            }
            if (newState != mState) {
                if constexpr(!std::is_same<StateExitCallback, void>::value) {
                    StateExitCallback::exit(mState);
                }
                mState = newState;
                if constexpr(!std::is_same<StateEntryCallback, void>::value) {
                    StateEntryCallback::enter(mState);
                }
            }
            if constexpr(!std::is_same<ProcessCallback, void>::value) {
                ProcessCallback::process(mState);
            }
        }        
    };
    
};