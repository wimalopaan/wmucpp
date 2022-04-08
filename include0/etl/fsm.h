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
#include <cstddef>
#include <concepts>

#include <external/solutions/tick.h>

#include <mcu/pgm/pgmarray.h>

#include "meta.h"

namespace FSM {
    namespace Timed {
        namespace Concepts {
            template<typename T, typename ValueType>
            concept State = requires(T) {
                            typename T::timer;
            {T::template process<void>(ValueType{})} -> std::same_as<void>;
            {T::onEnter()} -> std::same_as<void>;
            {T::onExit()} -> std::same_as<void>;
        };
        }
        
        template<typename StartState, typename StateList, typename ValueType = std::byte> 
        requires(Concepts::State<StartState, ValueType>)
        struct StateProcessor;
        
        template<typename StartState, typename... States, typename ValueType> 
        requires(Concepts::State<StartState, ValueType> && (Concepts::State<States, ValueType> && ...))
        struct StateProcessor<StartState, Meta::List<States...>, ValueType> {
            using stateList = Meta::List<States...>;
            
            static_assert((sizeof...(States) > 1), "need at least two states");
            static_assert(Meta::is_set_v<stateList>, "all states must be different");
            static_assert(Meta::contains_v<stateList, StartState>, "start state must be element of state set");
            static_assert(Meta::all_same_front_v<Meta::List<typename States::timer ...>>, "timers must all be the same");
            
            using state_index_t = etl::typeForValue_t<sizeof...(States)>;        
            using sp = StateProcessor<StartState, stateList, ValueType>;
            
            using timer = StartState::timer;
            
            static inline constexpr void ratePeriodic() {
                ++mStateTicks;
            }
            
            static inline constexpr void on(const auto& timeout, auto f) {
                mStateTicks.on(timeout, f);
            }
            
            static inline constexpr void process(const ValueType& b) {
                Meta::visitAt<stateList>(mState, [&]<typename State>(Meta::Wrapper<State>){
                                             State::template process<sp>(b);
                                         });
            }                    
            template<typename ToState>
            static inline constexpr void toState() {
                const uint8_t oldState = mState;
                mState = Meta::index_v<stateList, ToState>;
                if (oldState != mState) {
                    Meta::visitAt<stateList>(oldState, [&]<typename S>(Meta::Wrapper<S>){
                                                 S::onExit();
                                             });                
                    Meta::visitAt<stateList>(mState, [&]<typename S>(Meta::Wrapper<S>){
                                                 S::onEnter();
                                             });
                    mStateTicks.reset();
                }
            }                    
            private:
            static inline state_index_t mState{Meta::index_v<stateList, StartState>};
            static inline External::Tick<timer> mStateTicks;
        };        
        
    }
    
    namespace Simple {
        namespace Concepts {
            template<typename T, typename ValueType>
            concept State = requires(T) {
            {T::template process<void>(ValueType{})} -> std::same_as<void>;
            {T::onEnter()} -> std::same_as<void>;
            {T::onExit()} -> std::same_as<void>;
        };
        }
        
        template<typename StartState, typename StateList, typename ValueType = std::byte> 
        requires(Concepts::State<StartState, ValueType>)
        struct StateProcessor;
        
        template<typename StartState, typename... States, typename ValueType> 
        requires(Concepts::State<StartState, ValueType> && (Concepts::State<States, ValueType> && ...))
        struct StateProcessor<StartState, Meta::List<States...>, ValueType> {
            using stateList = Meta::List<States...>;
            
            static_assert((sizeof...(States) > 1), "need at least two states");
            static_assert(Meta::is_set_v<stateList>, "all states must be different");
            static_assert(Meta::contains_v<stateList, StartState>, "start state must be element of state set");
            
            using state_index_t = etl::typeForValue_t<sizeof...(States)>;        
            using sp = StateProcessor<StartState, stateList, ValueType>;
            
            static inline constexpr void process(const ValueType b) {
//                const auto oldState = mState;
                Meta::visitAt<stateList>(mState, [&]<typename State>(Meta::Wrapper<State>){
                                             State::template process<sp>(b);
                                         });
//                if (oldState != mState) {
//                    Meta::visitAt<stateList>(oldState, []<typename S>(Meta::Wrapper<S>){
//                                                 S::onExit();
//                                             });                
//                    Meta::visitAt<stateList>(mState, []<typename S>(Meta::Wrapper<S>){
//                                                 S::onEnter();
//                                             });                
//                }
            }     
            
            template<typename ToState>
            static inline constexpr void toState() {
                mState = Meta::index_v<stateList, ToState>;
                ToState::onEnter();
            }                    
            
            private:
            static inline state_index_t mState{Meta::index_v<stateList, StartState>};
        };        
    }
    namespace Enum {
        namespace Concepts {
            template<typename T, typename ValueType, typename SI>
            concept State = requires(T, SI x) {
                            T::template process<void>(ValueType{});
            {T::onEnter()} -> std::same_as<void>;
            {T::onExit()} -> std::same_as<void>;
        };
        }
        
        template<typename SI, typename StartState, typename StateList, typename ValueType = std::byte> 
        requires(Concepts::State<StartState, ValueType, SI>)
        struct StateProcessor;
        
        template<typename SI, typename StartState, typename... States, typename ValueType> 
        requires(Concepts::State<StartState, ValueType, SI> && (Concepts::State<States, ValueType, SI> && ...))
        struct StateProcessor<SI, StartState, Meta::List<States...>, ValueType> {
            using stateList = Meta::List<States...>;
            
            static_assert((sizeof...(States) > 1), "need at least two states");
            static_assert(Meta::is_set_v<stateList>, "all states must be different");
            static_assert(Meta::contains_v<stateList, StartState>, "start state must be element of state set");
            
            using sp = StateProcessor<SI, StartState, stateList, ValueType>;
            
            static inline constexpr void process(const ValueType b) {
//                const auto oldState = mState;
                Meta::visitAtJT<stateList>(mState, [&]<typename State>(Meta::Wrapper<State>){
                                               State::template process<sp>(b);
                                           });
//                if (oldState != mState) {
//                    Meta::visitAtJT<stateList>(oldState, []<typename S>(Meta::Wrapper<S>){
//                                                   S::onExit();
//                                               });                
//                    Meta::visitAtJT<stateList>(mState, []<typename S>(Meta::Wrapper<S>){
//                                                   S::onEnter();
//                                               });                
//                }
            }
            template<typename> struct vhelper;
            template<template<auto> typename A, auto I>
            struct vhelper<A<I>> {
                inline static constexpr auto index = I;
            };
            
            template<typename ToState>
            static inline constexpr void toState() {
                mState = vhelper<ToState>::index;
                ToState::onEnter();
            }                    

//            static inline constexpr void toState(const SI state) {
//                mState = state;
//            }
            private:
            static inline SI mState{0};
        };        
    }
}
