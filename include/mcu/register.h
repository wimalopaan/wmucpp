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
    
    void inline set(BitType v) {
        hwRegister = static_cast<value_type>(v);
    }
    template<BitType F>
    void inline set() {
        hwRegister = static_cast<value_type>(F);
    }
    template<BitType F>
    void inline setPartial(BitType v) {
        hwRegister = (hwRegister & static_cast<value_type>(~F)) | static_cast<value_type>(v);
    }
    template<BitType F>
    void inline add() {
        hwRegister |= static_cast<value_type>(F);
    }
    template<BitType F>
    void inline clear() {
        hwRegister &= ~static_cast<value_type>(F);
    }
    template<BitType Mask>
    inline BitType get() {
        return static_cast<BitType>(hwRegister & static_cast<value_type>(Mask));
    }
    template<uint8_t Mask>
    inline BitType get() {
        return static_cast<BitType>(hwRegister & Mask);
    }
    template<BitType F>
    bool inline isSet() {
        return hwRegister & static_cast<value_type>(F);
    }

private:
    volatile value_type hwRegister;
};

template<typename Component, typename Mode = ReadWrite, typename ValueType = uint8_t>
struct DataRegister {
    typedef Component component_type;
    typedef ValueType value_type;    

    // SFINAE
    template<typename Dummy = void, 
             typename Dummy2 = typename std::enable_if<std::is_same<Mode, ReadWrite>::value, Dummy>::type>
    inline volatile value_type& operator*() {
        return hwRegister;
    }
    
    inline const volatile value_type& operator*() const {
        return hwRegister;
    }
    
private:    
    volatile value_type hwRegister;
};

}
