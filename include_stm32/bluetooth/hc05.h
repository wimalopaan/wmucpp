/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstddef>
#include <chrono>

#include <etl/fixedvector.h>
#include <etl/ranged.h>

namespace External {
    namespace AT {
        template<uint8_t Size>
        struct Response {
            using buffer_t = etl::FixedVector<std::byte, Size>;
            
            enum class State : uint8_t {Init, GotO, GotK, GotPlus, Name, GotColon, Value, End};
            
            static inline bool isActive() {
                return mActive;
            }  
            static inline void activate(const bool on) {
                mActive = on;
            }
            static inline void clear() {
                mName.clear();
                mValue.clear();
                mBytes = 0;
            }
            static inline void push(const std::byte b) {
                if (mActive) {
                    ++mBytes;
                    switch(mState) {
                        case State::Init:
                            if (b == std::byte{'O'}) {
                                mState = State::GotO;
                            }
                            else if (b == std::byte{'+'}) {
                                mState = State::GotPlus;
                            }
                            break;
                    case State::GotO:
                        if ((b == std::byte{'\r'}) || (b == std::byte{'\n'})) {
                            mState = State::End;
                        }
                        else if (b == std::byte{'K'}) {
                            mState = State::GotK;
                        }
                        else {
                            mState = State::Init;
                        }
                        break;
                    case State::GotK:
                        if ((b == std::byte{'\r'}) || (b == std::byte{'\n'})) {
                            mState = State::End;
                        }
                        else {
                            mState = State::Init;
                        }
                        break;
                    case State::GotPlus:
                        if ((b == std::byte{'\r'}) || (b == std::byte{'\n'})) {
                            mState = State::End;
                        }
                        else {
                            mName.clear();
                            mValue.clear();
                            mName.push_back(b);;
                            mState = State::Name;
                        }
                        break;
                    case State::Name:
                        if ((b == std::byte{'\r'}) || (b == std::byte{'\n'})) {
                            mState = State::End;
                        }
                        else if (b == std::byte{':'}) {
                            mState = State::GotColon;
                        }
                        else {
                            mName.push_back(b);
                        }
                        break;
                    case State::GotColon:
                        if ((b == std::byte{'\r'}) || (b == std::byte{'\n'})) {
                            mState = State::End;
                        }
                        else {
                            mValue.push_back(b);;
                            mState = State::Value;
                        }
                        break;
                    case State::Value:
                        if ((b == std::byte{'\r'}) || (b == std::byte{'\n'})) {
                            mState = State::End;
                        }
                        else {
                            mValue.push_back(b);;
                        }
                        break;
                    case State::End:
                        if ((b == std::byte{'\r'}) || (b == std::byte{'\n'})) {
                            mState = State::Init;
                        }
                        else if (b == std::byte{'O'}) {
                            mState = State::GotO;
                        }
                        break;
                    }
                }
            }  
            static inline buffer_t& name() {
                return mName;                
            } 
            static inline buffer_t& value() {
                return mValue;                
            } 
            static inline uint8_t bytes() {
                return mBytes;                
            } 
        private:
            static inline uint8_t mBytes{0};
            static inline bool mActive{false};
            static inline buffer_t mName;
            static inline buffer_t mValue;
            static inline State mState{State::Init};
        };
    }
    
    using namespace std::literals::chrono_literals;
    using namespace etl::literals;
    
    template<auto Name, typename Dev, typename EN, typename Pwr, typename Timer, typename Term = void>
    struct HC05 {
        using pa = Dev::protocoll_adapter_type;
        using pa_buffer = pa::buffer_t;
        
        static inline auto& atName  = pa_buffer::name();
        static inline auto& atValue = pa_buffer::value();
        
        using out = etl::basic_ostream<Dev>;
        using term = Term;
        
        static inline constexpr External::Tick<Timer> initTicks{100ms};
        static inline constexpr External::Tick<Timer> waitTicks{500ms};
        
        enum class State : uint8_t {Init, Power, AT, QueryName, NameResult, SetName, Comm};
        
        enum class Event : uint8_t {None, SetName, CommunicationMode};
        
        static inline void init() {
            // Dev::template init<AVR::BaudRate<9600>>();
            EN::init();
            Pwr::init();
        }
        
        static inline void periodic() {
            Dev::periodic();
        }
        
        static inline void event(const Event e) {
            mEvent = e;
        }
        
        static inline void ratePeriodic() {
            ++mStateTicks;
            const auto oldState = mState;
            switch(mState) {
            case State::Init:
                mStateTicks.on(initTicks, []{
                   mState = State::Power; 
                });
                break;
            case State::Power:
//                mStateTicks.on(waitTicks, []{
//                   mState = State::AT; 
//                });

                // std::exchange
                if (mEvent == Event::SetName) {
                    mState = State::AT;
                    mTargetState = State::SetName;
                }
                else if (mEvent == Event::CommunicationMode) {
                    mState = State::Comm;
                }
                break;
            case State::AT:
                mStateTicks.on(waitTicks, []{
//                   mState = State::QueryName; 
                    mState = mTargetState;
                });
                break;
            case State::QueryName:
                mStateTicks.on(initTicks, []{
                   mState = State::NameResult; 
                });
                break;
            case State::NameResult:
                mStateTicks.on(initTicks, []{
                   mState = State::SetName; 
                });
                break;
            case State::SetName:
                mStateTicks.on(initTicks, []{
                   mState = State::Comm; 
                });
                break;
            case State::Comm:
                break;
            }
            if (oldState != mState) {
                mStateTicks.reset();
                switch(mState) {
                case State::Init:
                    IO::outl<term>("H Init");
                    break;
                case State::Power:
                    IO::outl<term>("H Pwr");
                    EN::inactivate();
                    Pwr::activate();
                    break;
                case State::AT:
                    IO::outl<term>("H AT");
                    EN::activate();
                    break;
                case State::QueryName:
                    IO::outl<term>("H QN");
                    pa_buffer::clear();
                    pa_buffer::activate(true);
                    IO::outl<out>("AT+NAME?");
                    break;
                case State::NameResult:
                    IO::outl<term>("H NR: ", atName.size(), " , ", atValue.size(), " , ", pa_buffer::bytes());
                    for(uint8_t i{}; i < atName.size(); ++i) {
                        IO::out<term>(atName[i]);
                    }
                    IO::out<term>('=');
                    for(uint8_t i{}; i < atValue.size(); ++i) {
                        IO::out<term>(atValue[i]);
                    }
                    IO::outl<term>();
                    break;
                case State::SetName:
                    pa_buffer::clear();
                    pa_buffer::activate(true);
                    IO::outl<out>("AT+NAME=CruiseCtrl");
                    break;
                case State::Comm:
                    IO::outl<term>("H Comm");
                    EN::inactivate();
                    pa_buffer::activate(false);
                    break;
                }
            }
        }
    private:
        static inline Event mEvent{Event::None};
        
        // static inline Event event() {
        //     Event result{Event::None};
        //     std::swap(mEvent, result);
        //     return result;
        // }
        
        static inline State mTargetState{State::Init};
        static inline State mState{State::Init};
        static inline External::Tick<Timer> mStateTicks;        
    };
}
