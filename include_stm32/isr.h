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

#include <concepts>
#include <csignal>
#include "atomic.h"

namespace Mcu::Arm::Interrupt {
    
    struct non_atomic {};
    
    template<typename T>
    class volatile_atomic;
    
    template<std::integral T>
    requires (sizeof(T) > sizeof(sig_atomic_t))
    class volatile_atomic<T> {
    public:
        constexpr volatile_atomic() = default;
        volatile_atomic(const volatile_atomic&) = delete;
        void operator=(const volatile_atomic&) = delete;
        
        void operator=(const T v) {
            mValue = v;
        }
        void operator=(const T v) volatile {
            Atomic::DisableInterruptsRestore di;
            mValue = v;
        }
        void operator++() volatile {
            Atomic::DisableInterruptsRestore di;
            mValue = mValue + 1; // <> read-modify-write: possible lost-update -> DisableInterrupts
        }
        void operator++() {
            ++mValue;
        }
        void operator+=(const T v) {
            mValue += v;
        }
        void operator+=(const T v) volatile {
            Atomic::DisableInterruptsRestore di;
            mValue = mValue + v; // <> read-modify-write: possible lost-update -> DisableInterrupt
        }
        operator T() const {
            return mValue;
        }
        operator T() const volatile {
            Atomic::DisableInterruptsRestore di;
            return mValue;
        }
        const volatile T& value() const volatile {
            Atomic::DisableInterruptsRestore di;
            return mValue;            
        }
        const T& value() const {
            Atomic::DisableInterruptsRestore di;
            return mValue;            
        }
        volatile T& value(non_atomic) volatile {
            return mValue;
        }
        const volatile T& value(non_atomic) const volatile {
            return mValue;
        }
    private:
        T mValue{}; 
    };
    
    template<typename T>
    requires (sizeof(T) <= sizeof(sig_atomic_t))
    class volatile_atomic<T> {
    public:
        constexpr volatile_atomic() = default;
        
        volatile_atomic(const volatile_atomic&) = delete;
        void operator=(const volatile_atomic&) = delete;
        
        void operator=(const T v) {
            mValue = v;
        }
        void operator=(const T v) volatile {
            mValue = v;
        }
        void operator++() {
            ++mValue; 
        }
        void operator++() volatile {
            T val;                                            
            do {                                                     
                val = __LDREXW(&mValue);  
                val += 1;
            } while ((__STREXW(val, &mValue)) != 0U); 
        }
        operator T() const {
            return mValue;
        }
        operator T() const volatile {
            return mValue;
        }
        void on(const T v, auto f) volatile {
            Atomic::DisableInterruptsRestore di;
            if (mValue == v) {
                f(mValue);
            }
        }
        void on(const T v, auto f) const volatile {
            Atomic::DisableInterruptsRestore di;
            const T l = mValue;
            if (l == v) {
                f(l);
            }
        }
        void on(const T v, non_atomic, auto f) const volatile {
            const T l = mValue;
            if (l == v) {
                f(l);
            }
        }
        void on(const T v, auto f) const {
            if (mValue == v) {
                f(mValue);
            }
        }
        void use(auto f) const volatile {
            const T v = mValue;
            f(v);
        }
        void use(auto f) volatile {
            f(mValue);
        }
    private:
        T mValue{}; 
    };
}
