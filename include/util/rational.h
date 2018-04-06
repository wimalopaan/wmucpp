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

#include "util/bits.h"

namespace Util {
    template<typename T>
    inline constexpr T gcd(T a, T b) {
        return b == 0 ? a : gcd(b, a % b);
    }
    
    template<typename T, uint64_t Nom, uint64_t Denom>
    struct RationalDivider {
        static_assert(Nom > 0, "Nominator must be greator 0");
        static_assert(Denom > 0, "Denominator must be greator 0");
        static_assert(Nom < Denom, "Nominator must be less than Denominator");
        inline static constexpr uint64_t GCD = gcd(Nom, Denom);
        inline static constexpr uint64_t N = Nom / GCD;
        inline static constexpr uint64_t D = Denom / GCD;
        typedef typename Util::enclosingType<T>::type U;
        struct DivisionData {
            uint8_t shifts = 0;
            U multiplierFull = 0;
            T multiplierTruncated = 0;
        };
        inline static constexpr DivisionData data = [](){
            DivisionData data;
            double factor = double(N) / D;
            
            for(uint8_t i = 0; i < Util::numberOfBits<U>(); ++i) {
                double d = 1.0 / (uint64_t(1) << (i + 1));
                if (d < factor) {
                    factor -= d;
                    data.multiplierFull |= (U(1) << (Util::numberOfBits<U>() - i - 1));
                }
            }  
            for(uint8_t i = 0; i < Util::numberOfBits<U>(); ++i) {
                if (Util::isSet<Util::MSB>(data.multiplierFull)) {
                    data.shifts = i;
                    break;
                }
                data.multiplierFull <<= 1;
            }        
            if (data.shifts > 0) {
                data.multiplierFull += (U(1) << (data.shifts - 1)); 
            }
            data.multiplierTruncated = (data.multiplierFull >> Util::numberOfBits<T>());
            return data;
        }();
        
        static_assert(data.shifts < Util::numberOfBits<T>());
        static_assert(data.multiplierFull != 0);
        static_assert(data.multiplierTruncated != 0);
        
        inline static constexpr T scale(T value) {
            if constexpr(std::is_same<T, uint8_t>::value) { // no explicit upcasting due to integral promotion
                return ((value * data.multiplierTruncated) / (std::numeric_limits<T>::module())) / (T(1) << data.shifts);
            }
            else {
                return ((U(value) * data.multiplierTruncated) / (std::numeric_limits<T>::module())) / (T(1) << data.shifts);
            }    
        }
    };
    
    template<typename T, uint64_t Nominator, uint64_t Denominator>
    class uint_scaled final {
    public:
        inline static constexpr uint64_t GCD = gcd(Nominator, Denominator);
        inline static constexpr uint64_t N = Nominator / GCD;
        inline static constexpr uint64_t D = Denominator / GCD;
        static_assert(D != 0);
        
        inline static constexpr uint8_t NumberOfBits = Util::numberOfBits<T>();
        
        typedef T type;
        typedef typename Util::TypeForValue<N>::type NT;
        typedef typename Util::TypeForValue<D>::type DT;
        
        constexpr uint_scaled(T v = 0) : value(v) {}
        
        T toInt() const {
            return RationalDivider<T, N, D>::scale(value);
        }
        const T& unscaled() const {
            return value;
        }
    private:
        T value = 0;
    };
    
} // !Util

template<typename T, uint64_t Nom1, uint64_t Denom1, uint64_t Nom2, uint64_t Denom2>
constexpr auto operator*(Util::uint_scaled<T, Nom1, Denom1> lhs, Util::uint_scaled<T, Nom2, Denom2> rhs) {
    constexpr uint64_t N = (Nom1 * Nom2) / Util::gcd(Nom1 * Nom2, Denom1 * Denom2);
    constexpr uint64_t D = (Denom1 * Denom2) / Util::gcd(Nom1 * Nom2, Denom1 * Denom2);
    typedef typename Util::enclosingType<T>::type U;
    return Util::uint_scaled<U, N, D>{typename Util::enclosingType<T>::type(lhs.unscaled() * rhs.unscaled())};
}
