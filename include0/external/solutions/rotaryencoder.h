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

#include <std/chrono>
#include <std/variant>
#include <std/utility>

#include <etl/rational.h>
#include <etl/concepts.h>
#include <etl/algorithm.h>

#include "mcu/common/concepts.h"

#include "tick.h"

namespace External {
    template<typename Pin1, typename Pin2, typename ValueType = uint8_t>
    class RotaryEncoder;

    template<AVR::Concepts::Pin Pin1, AVR::Concepts::Pin Pin2, typename... VTs>
    class RotaryEncoder<Pin1, Pin2, std::variant<VTs...>> {
    public:
        using value_type = std::variant<VTs...>;
        using ValueType = value_type;
        typedef Pin1 pin1_type;
        typedef Pin2 pin2_type;
        
        using type_list = Meta::List<VTs...>;
        using first_t = Meta::front<type_list>;
        
        static inline void init(const first_t& v = first_t{}) {
            mValue = v;
            Pin1::template dir<AVR::Input>();
            Pin2::template dir<AVR::Input>();
            Pin1::template pullup<true>();
            Pin2::template pullup<true>();
        }
        
        template<typename V>
        requires(Meta::contains_v<type_list, V>)
        static inline void set(const V& v) {
            mValue = v;
        }
        
        static inline void rateProcess() {
            const uint8_t newState = (Pin1::read() ? 2 : 0) + (Pin2::read() ? 1 : 0);    
            switch (mLastState) {
            case 0:
                if (newState == 1) {
                    mValue.visit([](auto& v){
                        ++v;
                    });
                }
                else if (newState == 2) {
                    mValue.visit([](auto& v){
                        --v;
                    });
                }
                break;
            case 1:
                if (newState == 0) {
                    mValue.visit([](auto& v){
                        --v;
                    });
                }
                break;
            case 2:
                if (newState == 0) {
                    mValue.visit([](auto& v){
                        ++v;
                    });
                }
                break;
            case 3:
                break;
            default:
                std::unreachable();
//                __builtin_unreachable();
//                assert(false);
            }
            mLastState = newState;
        }
        static inline const ValueType& value() {
            return mValue;
        }   
    private:
        inline static ValueType mValue{};
        inline static uint8_t mLastState{};
    };
    
    template<AVR::Concepts::Pin Pin1, AVR::Concepts::Pin Pin2, typename ValueType>
    class RotaryEncoder<Pin1, Pin2, ValueType> {
    public:
        typedef ValueType value_type;
        typedef Pin1 pin1_type;
        typedef Pin2 pin2_type;
        
        static void init(const ValueType& v = ValueType{}) {
            mValue = v;
            Pin1::template dir<AVR::Input>();
            Pin2::template dir<AVR::Input>();
            Pin1::template pullup<true>();
            Pin2::template pullup<true>();
        }
        
        static void start() {}

        static inline void rateProcess() {
            const uint8_t newState = (Pin1::read() ? 2 : 0) + (Pin2::read() ? 1 : 0);    
            switch (mLastState) {
            case 0:
                if (newState == 1) {
                    ++mValue;
                }
                else if (newState == 2) {
                    --mValue;
                }
                break;
            case 1:
                if (newState == 0) {
                    --mValue;
                }
                break;
            case 2:
                if (newState == 0) {
                    ++mValue;
                }
                break;
            case 3:
                break;
            default:
                std::unreachable();
//                __builtin_unreachable();
//                assert(false);
            }
            mLastState = newState;
        }
        static ValueType value() {
            return mValue;
        }   
    private:
        inline static ValueType mValue{};
        inline static uint8_t mLastState{};
    };
    
    namespace deprecated {
        template<typename Pin1, typename Pin2, typename ValueType = uint8_t, ValueType d = ValueType{}>
        class RotaryEncoder;
    
        template<AVR::Concepts::Pin Pin1, AVR::Concepts::Pin Pin2, typename... VTs, std::variant<VTs...> d>
        class RotaryEncoder<Pin1, Pin2, std::variant<VTs...>, d> {
            
        };
        
        template<AVR::Concepts::Pin Pin1, AVR::Concepts::Pin Pin2, typename ValueType, ValueType d>
        class RotaryEncoder<Pin1, Pin2, ValueType, d> {
        public:
            typedef ValueType value_type;
            typedef Pin1 pin1_type;
            typedef Pin2 pin2_type;
            static void init() {
                Pin1::template dir<AVR::Input>();
                Pin2::template dir<AVR::Input>();
                Pin1::template pullup<true>();
                Pin2::template pullup<true>();
            }
            static void start() {}
    
            static inline void rateProcess() {
                uint8_t newState = (Pin1::read() ? 2 : 0) + (Pin2::read() ? 1 : 0);    
                switch (mLastState) {
                case 0:
                    if (newState == 1) {
                        ++mValue;
                    }
                    else if (newState == 2) {
                        --mValue;
                    }
                    break;
                case 1:
                    if (newState == 0) {
                        --mValue;
                    }
                    break;
                case 2:
                    if (newState == 0) {
                        ++mValue;
                    }
                    break;
                case 3:
                    break;
                default:
                    assert(false);
                }
                mLastState = newState;
            }
            static ValueType value() {
                return mValue;
            }   
        private:
            inline static ValueType mValue{d};
            inline static uint8_t mLastState = 0;
        };
    }
}
