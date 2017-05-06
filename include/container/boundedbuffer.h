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

template<typename ValueType, uint8_t Capacity> 
class BoundedBuffer final {
public:
    static constexpr uint8_t capacity = Capacity;
    static constexpr uint8_t length = Capacity;
    
    typedef ValueType value_type;
    uint8_t size() const volatile {
        return mSize;
    }
    uint8_t size() const {
        return mSize;
    }
    const ValueType* begin() const {
        return &mData[0];
    }
    const volatile ValueType* begin() const volatile {
        return &mData[0];
    }
    const ValueType* end() const {
        return &mData[mSize];
    }
    const volatile ValueType* end() const volatile {
        return &mData[mSize];
    }
    ValueType* begin() {
        return &mData[0];
    }
    volatile ValueType* begin() volatile {
        return &mData[0];
    }
    ValueType* end() {
        return &mData[mSize];
    }
    volatile ValueType* end() volatile {
        return &mData[mSize];
    }
    bool insert(const ValueType& item) {
        if (mSize < Capacity) {
            mData[mSize++] = item;
            return true;
        }
        return false;
    }
    bool insert(const ValueType& item) volatile {
        if (mSize < Capacity) {
            mData[mSize++] = item;
            return true;
        }
        return false;
    }
    void clear() {
        mSize = 0;
    }
    void clear() volatile {
        mSize = 0;
    }
    const ValueType& operator[](uint8_t index) const {
        assert(index < mSize);
        return mData[index];
    }
    const volatile ValueType& operator[](uint8_t index) const volatile {
        assert(index < mSize);
        return mData[index];
    }
    private:
    ValueType mData[Capacity];
    uint8_t mSize = 0;
};
