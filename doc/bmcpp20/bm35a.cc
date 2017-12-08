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

#define NDEBUG

#include "mcu/avr8.h"
#include "units/percent.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using namespace std::literals::quantity;

volatile uint8_t y = 42;
volatile std::percent z = 0_ppc;

namespace detail {
    template<typename T>
    class LookupScaler {
    private:
        typedef typename Util::enclosingType<T>::type E;
        inline static constexpr auto lf = [](){
            std::array<E, std::numeric_limits<T>::max() + 1> data;
            for(E i = 1; i <= std::numeric_limits<T>::max(); ++i) {
                data[i] = (100 * 256) / i;
            }
            return data;
        }();
    public:
        static std::percent scale(T value, T min, T max) {
            typedef typename Util::enclosingType<E>::type R;
            return std::percent{(uint8_t)(((value - min) * (R)lf[max - min]) / 256)};
        }
    };
    template<typename T, T min, T max>
    class LookupScalerFixed {
    private:
        typedef typename Util::enclosingType<T>::type E;
        inline static constexpr auto lf = [](){
            std::array<E, std::numeric_limits<T>::max() + 1> data;
            for(E i = 1; i <= std::numeric_limits<T>::max(); ++i) {
                data[i] = ((i - min) * 100) / (max - min);
            }
            return data;
        }();
    public:
        static std::percent scale(T value) {
            return std::percent{(uint8_t)lf[value]};
        }
    };
} //!detail

template<typename T = uint8_t>
std::percent lookupScale(T value, T min, T max) {
    typedef typename std::remove_cv_t<T> U;
    if (value < min) {
        return std::percent{0U};
    }
    else if (value > max) {
        return std::percent{100U};
    }
    return detail::LookupScaler<U>::scale(value, min, max);
}

template<uint64_t min = 0, uint64_t max = std::numeric_limits<uint64_t>::max(), typename T = uint8_t>
std::percent lookupScale(T value) {
    typedef typename std::remove_cv_t<T> U;
    if (value < min) {
        return std::percent{0U};
    }
    else if (value > max) {
        return std::percent{100U};
    }
    return detail::LookupScalerFixed<U, U(min), U(max)>::scale(value);
}

int main() {
//    y = 42;

    {
        z = lookupScale(y, uint8_t(0), uint8_t(255));
    }
    
    while(true) {}
}

template<typename L>
void assertFunction(const PgmStringView&, const PgmStringView&, L) noexcept {
    while(true) {}
}
