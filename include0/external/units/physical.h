/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>
#include <type_traits>
#include <ratio>
#include <chrono>
#include <limits>

#include <etl/concepts.h>
#include <etl/type_traits.h>
#include <etl/outfwd.h>


namespace External {
    namespace Units {
        
        using namespace std;
        using namespace std::chrono;
        
        template<typename Representation, typename Scale = ratio<1,1>>
        struct voltage {
            using value_type = Representation;
            using divider_type = Scale;
            Representation value = 0;
        };
        
        template<typename R, typename S>
        inline bool operator>(const voltage<R,S>& lhs, const voltage<R,S>& rhs) {
            return lhs.value > rhs.value;
        }
        template<typename R, typename S>
        inline bool operator<(const voltage<R,S>& lhs, const voltage<R,S>& rhs) {
            return lhs.value < rhs.value;
        }
        
        using decivolt = voltage<uint16_t, ratio<1,10>>;
        using centivolt = voltage<uint16_t, ratio<1,100>>;
        using millivolt = voltage<uint16_t, ratio<1,1000>>;

        template<typename Representation, typename Scale = ratio<1,1>>
        struct ampere {
            using value_type = Representation;
            using divider_type = Scale;
            Representation value = 0;
        };

        template<typename R, typename S>
        inline bool operator>(const ampere<R,S>& lhs, const ampere<R,S>& rhs) {
            return lhs.value > rhs.value;
        }
        template<typename R, typename S>
        inline bool operator<(const ampere<R,S>& lhs, const ampere<R,S>& rhs) {
            return lhs.value < rhs.value;
        }
        
        using milliampere = ampere<uint16_t, ratio<1,1000>>;

        template<typename Representation, typename Scale = ratio<1,1>>
        struct celsius {
            using value_type = Representation;
            using divider_type = Scale;
            Representation value = 0;
        };
        template<typename R, typename S>
        inline bool operator>(const celsius<R,S>& lhs, const celsius<R,S>& rhs) {
            return lhs.value > rhs.value;
        }
        template<typename R, typename S>
        inline bool operator<(const celsius<R,S>& lhs, const celsius<R,S>& rhs) {
            return lhs.value < rhs.value;
        }
        
        
        template<typename Representation, typename Divider = ratio<1,1>>
        struct frequency;
        
        using hertz = frequency<uint32_t, ratio<1, 1>>;
        using megahertz = frequency<uint8_t, ratio<1, 1000000>>;
        
        template<>
        struct frequency<uint32_t> {
            typedef ratio<1, 1> divider_type;
            typedef uint32_t value_type;
            
            const uint32_t value = 0;
        };
        
        template<>
        struct frequency<uint8_t, ratio<1, 1000000>> {
            typedef ratio<1, 1000000> divider_type;
            typedef uint8_t value_type;
            
            const uint8_t value = 0;
            
            constexpr operator hertz() const{
                return {static_cast<uint32_t>(value * megahertz::divider_type::denom)};
            }
        };
        

        template<etl::Concepts::Stream Stream>
        inline void out_impl(const hertz& f) {
            etl::out<Stream>(f.value);
            etl::out<Stream>("Hz"_pgm);
        }
        template<etl::Concepts::Stream Stream>
        inline void out_impl(const megahertz& f) {
            etl::out<Stream>(f.value);
            etl::out<Stream>("MHz"_pgm);
        }

        
        template<typename Rep, typename Div, etl::Concepts::Integral I>
        constexpr frequency<Rep, Div> operator*(I i, const frequency<Rep, Div>& f) {
            return {f.value * i};
        }
        template<typename Rep, typename Div, etl::Concepts::Integral I>
        constexpr frequency<Rep, Div> operator*(const frequency<Rep, Div>& f, I i) {
            return {f.value * i};
        }
        
        template<typename Rep, typename Div>
        constexpr uint32_t operator/(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
        {
            return fl.value / fr.value;
        }
        
        template<typename Rep, typename Div>
        constexpr bool operator<(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
        {
            return fl.value < fr.value;
        }
        
        template<typename Rep, typename Div>
        constexpr bool operator>=(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
        {
            return !(fl.value < fr.value);
        }
        
        template<typename Rep, typename Div>
        constexpr bool operator<=(const frequency<Rep, Div>& fl, const frequency<Rep, Div>& fr)
        {
            return fl.value <= fr.value;
        }
        
        template<typename Rep, typename Div, etl::Concepts::Integral I>
        constexpr frequency<Rep, Div> operator/(const frequency<Rep, Div>& fl, I d) {
            return {(Rep)(fl.value / d)};
        }
        
        template<typename Rep, typename Div>
        constexpr bool operator==(const frequency<Rep, Div>& lhs, const frequency<Rep, Div>& rhs) {
            return lhs.value == rhs.value;
        }
        
        template<typename Rep, typename Div>
        constexpr bool operator!=(const frequency<Rep, Div>& lhs, const frequency<Rep, Div>& rhs) {
            return !(lhs == rhs);
        }
        
        template<typename Rep, typename Per, typename FRep, typename FDiv>
        constexpr uint32_t operator*(const duration<Rep, Per>& dt, const frequency<FRep, FDiv>& f) {
            return ((uint64_t)dt.value * f.value * FDiv::denom) / Per::denom;
        }
        
        template<typename Rep, typename Div, etl::Concepts::Integral I>
        constexpr duration<Rep, Div> operator*(const duration<Rep, Div>& fl, I m) {
            return {fl.value * m};
        }

        constexpr microseconds operator/(uint16_t v, const hertz& f) {
            return microseconds{(uint16_t)(((uint64_t)v * microseconds::period_type::denom) / f.value)};
        }
        
        constexpr hertz operator/(uint16_t v, const microseconds& f) {
            uint32_t x = v;
            uint32_t fl = f.value;
            return {(uint32_t)((x * microseconds::period_type::denom) / fl)};
        }
        constexpr hertz operator/(uint16_t v, const milliseconds& f) {
            uint32_t x = v;
            uint32_t fl = f.value;
            return {(uint32_t)((x * milliseconds::period_type::denom) / fl)};
        }
        
        template<typename T>
        constexpr milliseconds duration_cast(const microseconds&);
        
        template<>
        constexpr milliseconds duration_cast<milliseconds>(const microseconds& us) {
            return milliseconds{(uint16_t)((us.value + 500) / 1000)};
        }
        
        template<typename T>
        constexpr microseconds duration_cast(const milliseconds&);
        template<>
        constexpr microseconds duration_cast<microseconds>(const milliseconds& ms) {
            return microseconds{(uint16_t)(ms.value * 1000)};
        }
        
        class RPM {
        public:
            constexpr RPM() : mValue(numeric_limits<uint16_t>::max()) {}
            explicit constexpr RPM(uint16_t v) : mValue{v} {}
            explicit constexpr RPM(const hertz& f) : mValue(f.value * 60) {}
            
            constexpr explicit operator bool() const {
                return mValue != numeric_limits<uint16_t>::max();        
            }
            constexpr uint16_t value() const {
                if (mValue < numeric_limits<uint16_t>::max()) {
                    return mValue;
                }
                else {
                    return 0;
                }
            }
        private:
            uint16_t mValue = 0;
        };
        
        template<auto f>
        inline static constexpr RPM timerValueToRPM(auto v) {
            return RPM((60 * f.value) / v);
        }
        
        template<etl::Concepts::Stream Stream>
        inline void out_impl(const RPM& r) {
            etl::out<Stream>(r.value());
            etl::out<Stream>("Upm"_pgm);
        }
        
        
        namespace literals {
            using hertz = External::Units::hertz;
            using megahertz = External::Units::megahertz;
            
            constexpr hertz operator""_Hz(unsigned long long v) {
                return hertz{static_cast<uint32_t>(v)};
            }
            
            constexpr megahertz operator""_MHz(unsigned long long v) {
                return megahertz{static_cast<uint8_t>(v)};
            }
            
        }
        
    }
}

