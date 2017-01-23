/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include "std/traits.h"
#include "std/algorithm.h"
#include "std/utility.h"
#include "util/util.h"
#include "util/static_container.h"

namespace AVR {

struct ReadWrite {};
struct ReadOnly{};

template<typename Component, typename BitType, typename ValueType = uint8_t>
struct ControlRegister {
    typedef Component component_type;
    typedef ValueType value_type;    
    typedef BitType bit_type;
    
    template<typename... Flags>
    void inline set(Flags... f) {
        static_assert((std::is_same<bit_type, Flags>::value && ... && true), "wrong flag type");
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        value_type v = (static_cast<value_type>(f) | ... | 0);
        value_type n = Util::numberOfOnes(v);        
        assert(n == sizeof... (Flags));
        hwRegister = v;
    }

    template<typename F, F... FF>
    void inline set(Util::static_container<F, FF...>) {
        set<FF...>();
    }
    
    template<BitType... Flags>
    void inline set() {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (static_cast<value_type>(Flags) | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        hwRegister = v;
    }

    template<BitType... Flags>
    void inline add() {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (static_cast<value_type>(Flags) | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        hwRegister |= v;
    }

    template<BitType... Flags>
    void inline clear() {
        static_assert(sizeof...(Flags) <= 8, "too much flags");
        constexpr auto v = (static_cast<value_type>(Flags) | ... | 0);
        constexpr auto n = Util::numberOfOnes(v);        
        static_assert(n == sizeof... (Flags), "use different flags");
        hwRegister &= ~v;
    }
    
    volatile value_type hwRegister;
};

template<typename Component, typename Mode = ReadWrite, typename ValueType = uint8_t>
struct DataRegister {
    typedef Component component_type;
    typedef ValueType value_type;    

    // SFINAE
    template<typename Dummy = void, 
             typename Dummy2 = typename std::enable_if<std::is_same<Mode, ReadWrite>::value, Dummy>::type>
    inline void operator=(value_type v) {
        hwRegister = v;
    }
    
    volatile value_type hwRegister;
};

}
