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
#include "std/types.h"
#include "util/dassert.h"
#include "util/util.h"
#include "util/rational.h"

namespace std {
    struct percent {
        constexpr explicit percent(uint8_t p) : mValue(p) {
            assert(mValue <= 100);
        }
        constexpr percent(const percent& o) : mValue(o.mValue) {}
        percent(const volatile percent& o) : mValue(o.mValue) {}
        
        constexpr uint8_t value() const {
            return mValue;
        }
        void operator=(const percent& rhs) volatile {
            mValue = rhs.mValue;
        };
    private:
        uint8_t mValue = 0;
    };
    
    constexpr bool operator==(std::percent lhs, std::percent rhs) {
        return lhs.value() == rhs.value();
    }
    constexpr bool operator!=(std::percent lhs, std::percent rhs) {
        return !(lhs == rhs);
    }
    constexpr bool operator>(std::percent lhs, std::percent rhs) {
        return lhs.value() > rhs.value();
    }
    constexpr bool operator<(std::percent lhs, std::percent rhs) {
        return lhs.value() < rhs.value();
    }
    
    namespace literals {
        namespace quantity {
            constexpr std::percent operator"" _ppc(unsigned long long v) {
                return std::percent{static_cast<uint8_t>(v)};
            }
        }
    }
    
    template<std::Unsigned T>
    constexpr percent scale(const T& value, const std::remove_volatile_t<T> min, const std::remove_volatile_t<T> max) {
        if (value < min) {
            return std::percent{0u};
        }
        else if (value > max) {
            return std::percent{100u};
        }
        else {
            return std::percent{(uint8_t)((static_cast<typename Util::enclosingType<std::remove_volatile_t<T>>::type>(value - min) * 100u) / (max - min))};
        }
    }
    
    template<typename T, T L, T U>
    constexpr percent scale(const uint_ranged<T, L, U>& value) {
        return percent{(uint8_t)Util::RationalDivider<typename std::remove_cv<T>::type, 100, U-L>::scale(value.toInt() - L)};
    }
    template<MCU::RegisterType T, T L, T U>
    constexpr percent scale(uint_ranged<T, L, U> value) {
        return percent{(uint8_t)Util::RationalDivider<typename std::remove_cv<T>::type, 100, U-L>::scale(value.toInt() - L)};
    }
//    template<uint8_t L, uint8_t U>
//    constexpr percent scale(uint_ranged<uint8_t, L, U> value) {
//        return percent{(uint8_t)Util::RationalDivider<uint8_t, 100, U-L>::scale(value.toInt() - L)};
//    }
//    template<typename T, T L, T U>
//    constexpr percent scale(const volatile uint_ranged<T, L, U>& value) {
//        return percent{(uint8_t)Util::RationalDivider<typename std::remove_cv<T>::type, 100, U-L>::scale(value.toInt() - L)};
//    }
    
    //[[depracated("experimental")]] constexpr percent scale(uint8_t value) {
    //    uint16_t v = (value << 8);
    //    return std::percent{(uint8_t)(((v / 4u) + (v / 8u) + (v / 64u)) >> 8)};
    //}
    
    template<typename T, T min = 0, T max = std::numeric_limits<T>::max()>
    class FastDownScaler final {
        static_assert(((max - min) > 100), "wrong range");
    private:
        inline static constexpr uint8_t maximumNumberOfDivisions = sizeof(T) * 8 - 1 ;
        struct ScaleData {
            uint8_t numberOfDivisions;
            std::array<T, maximumNumberOfDivisions> divisions;
        };
        inline static constexpr auto data = []() {
            double factor = 100.0 / (max - min);
            std::array<T, maximumNumberOfDivisions> divisions;
            uint8_t d = 0;
            for(uint8_t i = 0; i < divisions.size; ++i) {
                uint64_t divisor = ((uint64_t)(1) << (i + 1));
                double f = 1.0 / divisor;
                if (f < factor) {
                    divisions[d++] = divisor;
                    factor -= f;
                }
            }
            return ScaleData{d, divisions};        
        }();
        template<uint8_t F>
        static constexpr T scale_r(T value) {
            if constexpr((F >= maximumNumberOfDivisions) || (data.divisions[F] == 0)) {
                (void) value;
                return 0;
            }
            else {
                // Wandlungsfehler
                return value / data.divisions[F] + scale_r<F+1>(value);
            }
        }
    public:
        inline static constexpr std::percent scale(T value) {
            return std::percent{(uint8_t)(scale_r<0>(value - min))};
        }
    };
    
    template<uint64_t min = 0, uint64_t max = std::numeric_limits<uint64_t>::max(), std::Unsigned T = uint8_t>
    std::percent fastScale(T value) {
        static_assert(max > min, "wrong range");
        typedef typename std::remove_cv<T>::type U;
        if (value < min) {
            return std::percent{0};
        }
        else if (value > max) {
            return std::percent{100};
        }
        if constexpr((max - min) > 100) {
            return FastDownScaler<U, U(min), U(max)>::scale(value);    
        }
        else {
            return std::percent{(uint8_t)((value - U(min)) * (100 / (max -min)))};   
        }
    }
    // todo: don't use this one
    template<typename T>
    constexpr T expand1(percent p, const T& min, const T& max) {
        return Util::RationalDivider<T, 1, 100>::scale(min + ((max - min) * p.value()));
    }
    template<typename T>
    constexpr T expand(percent p, const T& min, const T& max) {
        return min + ((max - min) * p.value()) / 100u;
    }
    
} // !std
