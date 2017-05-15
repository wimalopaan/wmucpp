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

namespace std {

template<typename E>
struct enable_bitmask_operators final {
    static constexpr const bool enable = false;
};

} // std

template<typename E>
constexpr
typename std::enable_if<std::enable_bitmask_operators<E>::enable,E>::type
operator|(E lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
                static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}

template<typename E>
constexpr
typename std::enable_if<std::enable_bitmask_operators<E>::enable,E>::type&
operator|=(E& lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return lhs = static_cast<E>(
                     static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}
template<typename E>
constexpr void operator|=(typename std::enable_if<std::enable_bitmask_operators<E>::enable, volatile E>::type& lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    lhs = static_cast<E>(
              static_cast<underlying>(lhs) | static_cast<underlying>(rhs)
              );
}

template<typename E>
constexpr
typename std::enable_if<std::enable_bitmask_operators<E>::enable,E>::type
operator&(E lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
                static_cast<underlying>(lhs) & static_cast<underlying>(rhs)
                );
}

template<typename E>
constexpr
typename std::enable_if<std::enable_bitmask_operators<E>::enable,E>::type&
operator&=(E& lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return lhs = static_cast<E>(
                     static_cast<underlying>(lhs) & static_cast<underlying>(rhs)
                     );
}

template<typename E>
constexpr void operator&=(typename std::enable_if<std::enable_bitmask_operators<E>::enable, volatile E>::type& lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    lhs = static_cast<E>(
              static_cast<underlying>(lhs) & static_cast<underlying>(rhs)
              );
}

template<typename E>
constexpr
typename std::enable_if<std::enable_bitmask_operators<E>::enable,E>::type
operator~(E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
                ~static_cast<underlying>(rhs)
                );
}

template<typename E>
constexpr
typename std::enable_if<std::enable_bitmask_operators<E>::enable,E>::type
operator^(E lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return static_cast<E>(
                static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs)
                );
}

template<typename E>
constexpr
typename std::enable_if<std::enable_bitmask_operators<E>::enable,E>::type&
operator^=(E& lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    return lhs = static_cast<E>(
                     static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs)
                     );
}

template<typename E>
constexpr void operator^=(typename std::enable_if<std::enable_bitmask_operators<E>::enable, volatile E>::type& lhs, E rhs){
    typedef typename std::underlying_type<E>::type underlying;
    lhs = static_cast<E>(
              static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs)
              );
}
