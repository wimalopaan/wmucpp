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

#include "config.h"
#include <cstdint>
#include "button.h"

namespace detail {
    namespace Button {
        template<typename Button>
        using port = typename Button::port_type;
        
        template<typename D> struct ControllerBase;
        
        template<template<auto, typename, typename, typename...> typename D, auto N, MCU::Interrupt Int, Util::NamedFlag activeLow, typename... Buttons>
        struct ControllerBase<D<N, Int, activeLow, Buttons...>> {
            typedef D<N, Int, activeLow, Buttons...> Derived;
            
            typedef Meta::List<Buttons...> button_list_raw;    
            typedef Meta::filter<Meta::nonVoid, button_list_raw> button_list;
            typedef std::make_index_sequence<N> CounterIndices;
            typedef std::make_index_sequence<Meta::size<button_list>::value> ButtonIndices;
            typedef typename Util::TypeForBits<N>::type DataType;
            typedef Meta::reverse<button_list> button_list_reverse;
            inline static constexpr bool useIsr = !std::is_same<Int, void>::value;
            using DataType_maybe_volatile = typename std::conditional<useIsr, typename std::add_volatile<DataType>::type, DataType>::type;

            inline static std::array<DataType, N> counter {};
            inline static DataType state{0};
            inline static DataType_maybe_volatile pressed{0};

            template<bool Q = !useIsr>
            static inline 
            typename std::enable_if<Q, void>::type
            periodic(const Util::Callable<DataType>& f) {
                Derived::update();
                f(pressed);
                pressed = DataType{0};
            }
            template<bool Q = !useIsr, typename... L>
            static inline 
            typename std::enable_if<Q, void>::type
            periodic(const L&... f) {
                static_assert(sizeof...(L) == Meta::size<button_list>::value);
                Derived::update();
                [&]<auto... II>(std::index_sequence<II...>){
                    (((pressed & (DataType{1} << II)) ? f() : (void)0),...);
                }(ButtonIndices{});
                pressed = DataType{0};
            }

            template<typename Button, bool Q = useIsr>
            static inline 
            typename std::enable_if<Q, bool>::type
            isPressed() {
                Scoped<DisbaleInterrupt<>> di;
                DataType v = pressed & (DataType{1} << Derived::template index<Button>::value);
                pressed ^= v;
                return v;
            }
            template<bool Q = useIsr>
            static inline 
            typename std::enable_if<Q, void>::type
            isr() {
                Derived::update();
            }
        private:
            static inline void increment(DataType which) {
                []<auto... II>(DataType w, std::index_sequence<II...>){
                    DataType c = w;
                    ((c = add(counter[II], c)), ...);
                }(which, CounterIndices{});
            }
            static inline DataType maxReached() {
                return []<auto... II>(std::index_sequence<II...>){
                    return (counter[II] & ...);
                }(CounterIndices{});
            }
            static inline DataType add(DataType& out, DataType value) {
                DataType carry = out & value;
                out ^= value;
                return carry;
            }
        };
    }
}

template<bool V>
struct ActiveLow : std::integral_constant<bool, V> {};

template<auto N, MCU::Interrupt Int, Util::NamedFlag activeLow, typename... Buttons>
struct ButtonControllerVertical : IsrBaseHandler<Int>, detail::Button::ControllerBase<ButtonControllerVertical<N, Int, activeLow, Buttons...>> {
    typedef detail::Button::ControllerBase<ButtonControllerVertical<N, Int, activeLow, Buttons...>> base;

    typedef typename base::DataType DataType;
    
    template<typename Button>
    struct index {
        typedef typename base::button_list_raw button_list_raw;
        inline static constexpr auto value = Meta::index<button_list_raw, Button>::value;
    };

    static inline void init() {
        typedef typename base::button_list button_list;        
        []<typename... BB>(Meta::List<BB...>) {
            (BB::init(),...);
        }(button_list{});
    }
private:
    static inline void update() {
        typedef typename base::button_list_reverse button_list_reverse;        
        DataType changed = []<typename... BB>(Meta::List<BB...>){
            DataType v{0};
                if constexpr(activeLow::value) {
                    ((v <<= 1u, v |= DataType{!BB::pin_type::isHigh()}), ...);
                }
                else {
                    ((v <<= 1u, v |= DataType{BB::pin_type::isHigh()}), ...);
                }
            return v;
        }(button_list_reverse{}) ^ base::state;
        base::increment(changed);
        DataType m = base::maxReached() & changed;
        base::state ^= m;
        base::pressed |= base::state & m;
    }
};

template<auto N, MCU::Interrupt Int, Util::NamedFlag activeLow, typename... Buttons>
requires Meta::all_same_front<Meta::transform<detail::Button::port, Meta::filter<Meta::nonVoid, Meta::List<Buttons...>>>>::value
struct ButtonControllerVertical<N, Int, activeLow, Buttons...> : IsrBaseHandler<Int>, detail::Button::ControllerBase<ButtonControllerVertical<N, Int, activeLow, Buttons...>> {
    typedef detail::Button::ControllerBase<ButtonControllerVertical<N, Int, activeLow, Buttons...>> base;

    typedef typename base::DataType DataType;
    typedef typename base::button_list button_list;
    typedef typename Meta::front<button_list>::port_type port_type;

    template<typename Button>
    struct index {
        inline static constexpr auto value = Button::pin_type::number;
    };
 
    template<typename> struct Parameter;
    template<typename... BB>
    struct Parameter<Meta::List<BB...>> {
        typedef AVR::PinSet<typename BB::pin_type...> pin_set;    
        inline static constexpr DataType mask = ((DataType{1} << index<BB>::value) | ...);
    };
    using parameter = Parameter<button_list>;
    
    using pin_set = typename parameter::pin_set;    
    inline static constexpr auto mask = parameter::mask;
    
    static inline void init() {
        pin_set::template dir<AVR::Input>();
        pin_set::template allPullup<NoDisableEnable>();
    }
private:
    static inline void update() {
        DataType changed {};
        if constexpr(activeLow::value) {
            changed = base::state ^ (~(DataType)port_type::read() & mask);
        } 
        else {
            changed = base::state ^ ((DataType)port_type::read() & mask);
        }
        base::increment(changed);
        DataType m = base::maxReached() & changed;
        base::state ^= m;
        base::pressed |= base::state & m;
    }
};
