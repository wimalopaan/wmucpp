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
#include "std/byte.h"
#include "util/util.h"
#include "util/static_container.h"

namespace AVR {

struct ReadWrite {};
struct ReadOnly{};
struct UnUsed{};

template<typename Component, typename BitType, typename ValueType = uint8_t>
struct ControlRegister final {
    typedef Component component_type;
    typedef ValueType value_type;    
    typedef BitType bit_type;
    
    ControlRegister() = delete;
    ControlRegister(const ControlRegister&) = delete;
    ControlRegister(ControlRegister&&) = delete;
    ControlRegister& operator=(const ControlRegister&) = delete;
    ControlRegister& operator=(ControlRegister&&) = delete;
    
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
struct DataRegister;

template<typename Component, typename ValueType>
struct DataRegister<Component, UnUsed, ValueType> final {
    typedef Component component_type;
    typedef ValueType value_type;    
    DataRegister() = delete;
    DataRegister(const DataRegister&) = delete;
    DataRegister(DataRegister&&) = delete;
    DataRegister& operator=(const DataRegister&) = delete;
    DataRegister& operator=(DataRegister&&) = delete;
private:    
    volatile value_type hwRegister; // needed to occupy space
};

template<typename Component, typename ValueType>
struct DataRegister<Component, ReadOnly, ValueType> final {
    typedef Component component_type;
    typedef ValueType value_type;    
    DataRegister() = delete;
    DataRegister(const DataRegister&) = delete;
    DataRegister(DataRegister&&) = delete;
    DataRegister& operator=(const DataRegister&) = delete;
    DataRegister& operator=(DataRegister&&) = delete;

    inline const volatile value_type& operator*() const {
        return hwRegister;
    }
private:    
    volatile value_type hwRegister;
};

template<typename Component, typename ValueType>
struct DataRegister<Component, ReadWrite, ValueType> final {
    typedef Component component_type;
    typedef ValueType value_type;    
    DataRegister() = delete;
    DataRegister(const DataRegister&) = delete;
    DataRegister(DataRegister&&) = delete;
    DataRegister& operator=(const DataRegister&) = delete;
    DataRegister& operator=(DataRegister&&) = delete;

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
