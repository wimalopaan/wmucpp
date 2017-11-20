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

#include "std/type_traits"
#include "std/memory"

#if __has_include(<avr/interrupt.h>)
# include <avr/interrupt.h>
#else
# define sei()
# define cli()
#endif

struct RestoreState {};
struct ForceOn {};

template<typename T = RestoreState>
struct EnableInterrupt {
    typedef T type;    
};
template<typename T = RestoreState>
struct DisbaleInterrupt {
    typedef T type;    
};

struct Transaction {};

template<typename T = Transaction, bool Active = true, typename F1 = void, typename F2 = void>
class Scoped;

template<bool Active>
class Scoped<EnableInterrupt<ForceOn>, Active> final
{
public:
    inline Scoped() {
        if constexpr(Active) {
            sei();
        }
    }
    inline ~Scoped() {
        if constexpr(Active) {
            cli();
        }
    }
};
template<bool Active>
class Scoped<EnableInterrupt<RestoreState>, Active> final
{
    inline static constexpr auto status = AVR::getBaseAddr<DefaultMcuType::Status>;
public:
    inline Scoped() {
        if constexpr(Active) {
            v = status()->value.value();
            sei();
        }
    }
    inline ~Scoped() {
        if constexpr(Active) {
            if (!std::toBool(DefaultMcuType::Status::Bits::globalIntEnable & v)) {
                cli();
            }
        }
    }
private:
    DefaultMcuType::Status::Bits v{0};
};

template<bool Active>
class Scoped<DisbaleInterrupt<ForceOn>, Active> final
{
public:
    inline Scoped() {
        if constexpr(Active) {
            cli();
        }
    }
    inline ~Scoped() {
        if constexpr(Active) {
            sei();
        }
    }
};
template<bool Active>
class Scoped<DisbaleInterrupt<RestoreState>, Active> final
{
    inline static constexpr auto status = AVR::getBaseAddr<DefaultMcuType::Status>;
public:
    inline Scoped()  {
        if constexpr(Active) {
            v = status()->value.value();
            cli();
        }
    }
    inline ~Scoped() {
        if constexpr(Active) {
            if (std::toBool(DefaultMcuType::Status::Bits::globalIntEnable & v)) {
                sei();
            }
        }
    }
private:
    DefaultMcuType::Status::Bits v{0};
};

template<typename F1, typename F2>
class Scoped<Transaction, true, F1, F2> final
{
public:
    inline Scoped(F1 f1, F2 f2) : f2(std::move(f2)) {
        f1();
    }
    Scoped(const Scoped&) = delete;
    Scoped(Scoped&&) = delete;
    
    Scoped& operator=(const Scoped&) = delete;
    Scoped& operator=(Scoped&&) = delete;
    
    inline ~Scoped() {
        f2();
    }
private:
    F2 f2;
};

// explicit deduction guide
template<typename F1, typename F2>
Scoped(F1 f1, F2 f2) -> Scoped<Transaction, true, F1, F2>;

