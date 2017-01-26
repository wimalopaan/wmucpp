/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016, 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "std/memory.h"

namespace std {

template<typename T>
class optional final {
public:
    optional() :
        mValid(false),
        mValue()
    {
    }
    optional(const T& value) :
        mValid(true),
        mValue(value)
    {
    }
    optional(const volatile T& value) :
        mValid(true),
        mValue(value)
    {
    }
    // todo: volatile?
    
//    optional(volatile T&& value) :
//        mValid(true),
//        mValue(std::move(value))
//    {
//    }
    explicit operator bool() const // safe-bool
    {
        return mValid;
    }
    explicit operator bool() // safe-bool
    {
        return mValid;
    }
    bool operator!() const {
        return !mValid;
    }
    T& operator*()
    {
        return mValue;
    }
    const T& operator*() const
    {
        return mValue;
    }
    T* operator->()
    {
        return &mValue;
    }
    const T* operator->() const
    {
        return &mValue;
    }
private:
    bool mValid = false;
    T mValue {};
};

}
