/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <stdint.h>

#include "hal/event.h"

namespace RadioClock {

enum class State : uint8_t {PreStart, Start, Sync, Clock, Error};
enum class Event : uint8_t {Reset, Start, DCFSync, DCFDecode, DCFError};

template<typename DcfDecoder, typename StateEntryCallback = void, typename StateExitCallback = void>
struct StateMachine final {
    StateMachine() = delete;
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
            break;
        case State::Start:
            if (event == Event::DCFSync) {
                newState = State::Sync;
            }
            else if (event == Event::Reset) {
                newState = State::PreStart;
            }
            break;
        case State::Sync:
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
            if (event == Event::Start) {
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
    }        
};

template<typename DcfDecoder, typename StateEntry = void, auto flash = nullptr>
class Clock final {
    using clockFSM = StateMachine<DcfDecoder, StateEntry>;
    
    struct DCFReceive0Handler : public EventHandler<EventType::DCFReceive0> {
        static bool process(std::byte) {
            if constexpr(flash) {
                flash(false);
                return true;
            }
            return false;
        }  
    };
    struct DCFReceive1Handler : public EventHandler<EventType::DCFReceive1> {
        static bool process(std::byte) {
            if constexpr(flash) {
                flash(true);
                return true;
            }
            return false;
        }  
    };
    struct DCFDecodeHandler : public EventHandler<EventType::DCFDecode> {
        static bool process(std::byte) {
            clockFSM::process(Event::DCFDecode);
            return true;
        }  
    };
    struct DCFSyncHandler : public EventHandler<EventType::DCFSync> {
        static bool process(std::byte) {
            clockFSM::process(Event::DCFSync);
            return true;
        }  
    };
    struct DCFErrorHandler : public EventHandler<EventType::DCFError> {
        static bool process(std::byte) {
            clockFSM::process(Event::DCFError);
            return true;
        }  
    };
    struct DCFParityHandler : public EventHandler<EventType::DCFParityError> {
        static bool process(std::byte) {
            clockFSM::process(Event::DCFError);
            return true;
        }  
    };
    struct ResetHandler : public EventHandler<EventType::RadioClockReset> {
        static bool process(std::byte) {
            clockFSM::process(Event::Reset);
            return true;
        }  
    };
    struct StartHandler : public EventHandler<EventType::RadioClockStart> {
        static bool process(std::byte) {
            clockFSM::process(Event::Start);
            return true;
        }  
    };
public:
    class HandlerGroup final : public EventHandlerGroup<
            DCFReceive0Handler, DCFReceive1Handler, DCFSyncHandler, DCFDecodeHandler,
            DCFErrorHandler, DCFParityHandler,
            ResetHandler, StartHandler> {
    };
    static void init() {
    }
private:
};

}