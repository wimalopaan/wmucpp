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

#include "atomic.h"

namespace etl {
    template<typename S> struct State;
    template<typename S>
    bool operator==(const State<S>& lhs, const State<S>& rhs);
    template<typename S>
    bool operator==(const State<S>& lhs, const S& rhs);
    
    template<typename S>
    struct State {
        friend bool operator==<>(const State<S>& lhs, const State<S>& rhs);
        friend bool operator==<>(const State<S>& lhs, const S& rhs);
        
        State() = default;
        State(const S& s) : mActual{s} {}
        
        State& operator=(const std::remove_cv_t<S>& next) {
            mPrevious = mActual;
            mActual = next;
            mFollowing = S::None;
            return *this;
        }
        void operator+=(const std::remove_cv_t<S>& following) {
            mFollowing = following;
        }
        operator S() const {
            return mActual;
        }
        S following() const {
            return mFollowing;
        }
        template<typename... SS>
        requires((std::is_same_v<std::remove_cv_t<S>, SS> && ...))
        bool contains(SS... s) {
            return Mcu::Arm::Atomic::access([&]{
                return ((s == mActual) || ...) || ((s == mFollowing) || ...);
            });
        } 
    private:
        S mActual = S::None;
        S mPrevious = S::None;
        S mFollowing = S::None;
    };
    template<typename S>
    bool operator==(const State<S>& lhs, const State<S>& rhs) {
        return lhs.mActual == rhs.mActual;
    }
    template<typename S>
    bool operator!=(const State<S>& lhs, const State<S>& rhs) {
        return !(lhs == rhs);
    }
    template<typename S>
    bool operator==(const State<S>& lhs, const S& rhs) {
        return lhs.mActual == rhs;
    }
}
