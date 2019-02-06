/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <iostream>
#include <iomanip>

#include <cstdint>

namespace etl {
    namespace detail {
        template<auto Bits>
        struct typeForBits {
            using type = typename std::conditional<(Bits <= 8), uint8_t, 
                             typename std::conditional<(Bits <= 16), uint16_t, 
                                 typename std::conditional<(Bits <= 32), uint32_t,
                                      typename std::conditional<(Bits <= 64), uint64_t, void>::type>::type>::type>::type;
        };
        template<auto V>
        struct typeForValue {
            using type = typename std::conditional<(V > std::numeric_limits<uint32_t>::max()), uint64_t, 
                                  typename std::conditional<(V > std::numeric_limits<uint16_t>::max()), uint32_t,
                                    typename std::conditional<(V > std::numeric_limits<uint8_t>::max()), uint16_t, uint8_t>::type>::type>::type;
        };
    
        template<typename T> struct enclosingType;

        template<> struct enclosingType<uint8_t> {
            typedef uint16_t type;
        };
        template<> struct enclosingType<uint16_t> {
            typedef uint32_t type;
        };
        template<> struct enclosingType<uint32_t> {
            typedef uint64_t type;
        };
        template<> struct enclosingType<int8_t> {
            typedef int16_t type;
        };
        template<> struct enclosingType<int16_t> {
            typedef int32_t type;
        };
        template<> struct enclosingType<int32_t> {
            typedef int64_t type;
        };
        
        template<typename T>struct fragmentType;
        
        template<> struct fragmentType<uint16_t> {
            typedef uint8_t type;
            static constexpr const uint8_t shift = 8;
        };
        
        template<> struct fragmentType<uint32_t> {
            typedef uint16_t type;
            static constexpr const uint8_t shift = 16;
        };
        template<> struct fragmentType<uint64_t> {
            typedef uint32_t type;
            static constexpr const uint8_t shift = 32;
        };
    }
    
//    template<typename E, typename = std::enable_if_t<std::enable_bitmask_operators_v<E>>>
//    inline constexpr bool toBool(E v) {
//        return static_cast<bool>(v);        
//    }
    
    template<typename T>
    inline constexpr uint8_t numberOfBits() {
        return sizeof(T) * 8;
    }

    template<uint64_t Bits>
    using typeForBits_t = typename detail::typeForBits<Bits>::type;  
    
    template<auto V>
    using typeForValue_t = typename detail::typeForValue<V>::type;
    
    template<typename T>
    using enclosingType_t = typename detail::enclosingType<T>::type;
    
    template<typename T>
    using fragmentType_t = typename detail::fragmentType<T>::type;    
    
//    using namespace etl::Concepts;
    
//    template<Integral T, uint8_t Base = 10>
//    inline constexpr uint8_t numberOfDigits() {
//        T v = std::numeric_limits<T>::max();
//        uint8_t number = 0;
//        while(v > 0) {
//            v /= Base;
//            ++number;
//        }
//        if (number == 0) {
//            number = 1;
//        }
//        if constexpr(std::is_signed<T>::value) {
//            number += 1;
//        }
//        return number;
//    }

    template<typename T, uint8_t Base = 10>
    requires (T::valid_bits > 0)
    inline constexpr uint8_t numberOfDigits() {
        double v = 1.0;
        for(uint8_t b = 0; b < T::valid_bits; ++b) {
            v /= 2.0;
        }
        uint8_t number = 0;
        while((v - (uint64_t)v) > 0.0) {
            number += 1;
            v *= 10.0;
        }
        return number;
    }
    
//    template<Unsigned T>
//    inline constexpr bool isPowerof2(T v) {
//        return v && ((v & (v - 1)) == 0);
//    }
    
//    template<typename Bit, typename T>
//    inline constexpr bool isSet(T v) {
//        if constexpr(std::is_same<T, std::byte>::value) {
//            return std::to_integer<uint8_t>(Bit::template Value<T>::value) & std::to_integer<uint8_t>(v);
//        }
//        else {
//            return Bit::template Value<T>::value & v;
//        }
//    }
    template<typename Bit, typename T>
    inline constexpr bool set(T& v) {
        return v |= Bit::template Value<T>::value;
    }
    
    struct MSB {
        template<typename T>
        struct Value {
            inline static constexpr const T value{T(1) << (numberOfBits<T>() - 1)};
        };
    };  
    struct LSB {
        template<typename T>
        struct Value {
            inline static constexpr const T value = 1;
        };
    };  
    template<typename T>
    inline constexpr uint8_t numberOfOnes(T x) {
        return (x != T{0}) ? T(x & T{0x01}) + numberOfOnes(T(x>>1u)) : 0u;
    }
}

namespace etl {
    using namespace std;
    
    template<typename T, uint8_t Bits>
    struct Fraction final {
        static_assert(std::is_unsigned<T>::value, "T must be unsigned type");
//        static_assert(etl::numberOfBits<T>() >= Bits, "type too small");
        typedef T value_type;
        inline static constexpr uint8_t valid_bits = Bits;
        constexpr explicit Fraction(T v) : value(v) {} 
        const T value = 0;
    };

    template<typename Type, uint8_t fractionalBits>
    class FixedPoint final {
        template<typename Stream> friend Stream& operator<<(Stream& o, const FixedPoint<Type, fractionalBits>& f);
    public:
        typedef Type value_type;
        typedef make_unsigned_t<Type> unsigned_type;
        typedef typeForBits_t<fractionalBits> fractional_type;
        // change
        inline static constexpr unsigned_type fractional_mask = (1 << fractionalBits) - 1;
        inline static constexpr unsigned_type integral_mask = ~((1 << fractionalBits) - 1);
        inline static constexpr uint8_t fractional_bits = fractionalBits;
        inline static constexpr uint8_t integer_bits = sizeof(Type) * 8 - fractionalBits;
        inline static constexpr value_type one = value_type{1} << fractional_bits;
//        typedef typeForBits_t<integer_bits> integral_type;
        
        inline constexpr FixedPoint(const FixedPoint& o) : mValue{o.mValue} {}

        inline constexpr FixedPoint(volatile const FixedPoint& o) : mValue{o.mValue} {}
        
        inline constexpr explicit FixedPoint(double v) : mValue(v * one) {}
    
        inline constexpr FixedPoint() = default;
        
        inline constexpr void operator=(FixedPoint rhs) volatile {
            mValue = rhs.mValue;
        }
        
        inline static FixedPoint<Type, fractionalBits> fromRaw(unsigned_type raw) {
            return FixedPoint<Type, fractionalBits>{raw};
        }
        
        inline unsigned_type integerAbs() const {
            if (mValue < 0) {
                return -integer();
            }
            return integer();
        }
        inline fractional_type fractionalAbs() const {
            return fractional();
        }
        // change
        inline value_type integer() const {
            return mValue / one;
        }
        // change
        inline value_type integer() const volatile {
            return mValue / one;
        }
        //change        
        inline fractional_type fractional() const volatile {
            return (fractional_type(mValue) & fractional_mask);
        }
        inline fractional_type fractional() const {
            return (fractional_type(mValue) & fractional_mask);
        }
        inline Fraction<fractional_type, fractionalBits> fraction() const {
            return Fraction<fractional_type, fractionalBits>(fractional() << ((sizeof(fractional_type) * 8) - fractional_bits));
        }
        inline const Type& raw() const {
            return mValue;
        }
        inline FixedPoint& operator/=(Type d) {
            mValue /= d;
            return *this;
        }
        inline FixedPoint& operator-=(const FixedPoint& d) {
            mValue -= d.mValue;
            return *this;
        }
        inline FixedPoint& operator+=(const FixedPoint& d) {
            mValue += d.mValue;
            return *this;
        }
        // todo: op++
        // todo: op--
    private:
        explicit FixedPoint(unsigned_type v) : mValue(v){}
        Type mValue = 0;
    };
    
    // todo: UDL-op for FP<uint16_t, 8>
    
    // change
    inline constexpr FixedPoint<int16_t, 4> operator"" _Qs11_4(long double v) {
        return FixedPoint<int16_t, 4>{(double)v};
    }
    //change
    inline constexpr FixedPoint<uint16_t, 8> operator"" _Qu8_8(long double v) {
        return FixedPoint<uint16_t, 8>{(double)v};
    }
    //change
    inline constexpr FixedPoint<int16_t, 12> operator"" _Qs3_12(long double v) {
        return FixedPoint<int16_t, 12>{(double)v};
    }
    
    template<typename T, auto Bits>
    inline FixedPoint<T, Bits> operator-(FixedPoint<T, Bits> lhs, const FixedPoint<T, Bits>& rhs) {
        return lhs -= rhs;
    }
    
    template<typename T, auto Bits, typename D>
    inline FixedPoint<T, Bits> operator/(FixedPoint<T, Bits> lhs, const D& rhs) {
        return lhs /= rhs;
    }
    
    template<typename T, uint8_t FB1, uint8_t FB2>
    inline FixedPoint<T, FB1> operator*(FixedPoint<T, FB1> lhs, FixedPoint<T, FB2> rhs) {
        enclosingType_t<T> p = lhs.raw() * rhs.raw();
        // change
        return FixedPoint<T, FB1>::fromRaw(p >> FB2);
    }
    
//    namespace detail {
//        template<etl::Concepts::Stream Stream, typename T, uint8_t Bits>
//        inline void out_impl(const Fraction<T, Bits>& f) {
//            array<Char, 1 + numberOfDigits<Fraction<T, Bits>>()> buffer; // dot + sentinel
//            ftoa(f, buffer);
//            out_impl<Stream>(buffer);
//        }
        
//        template<etl::Concepts::Stream Stream, Signed T, auto Bits>
//        inline void out_impl(const FixedPoint<T, Bits>& f) {
//            if (f.raw() < 0) {
//                out_impl<Stream>(Char{'-'});
//            }
//            out_impl<Stream>(f.integerAbs());
//            out_impl<Stream>(f.fraction());
//        }
    
//        template<etl::Concepts::Stream Stream, Unsigned T, uint8_t Bits>
//        inline void out_impl(const FixedPoint<T, Bits>& f) {
//            out_impl<Stream>(f.integerAbs());
//            out_impl<Stream>(f.fraction());
//        }
        
//    } // detail

}


using FP = etl::FixedPoint<int16_t, 12>;

volatile FP x{1.0};
volatile FP z;

int main() {
    using namespace etl;
    std::cout << FP::one << '\n';
    constexpr etl::FixedPoint<int16_t, 12> c{FP::one/3200.0};
    
    std::cout << "c: " << c.integer() << ' ' << c.fractional()/(double)decltype(c)::one << '\n';
  
    
    z = FP::fromRaw(3200);
    z = z * c;
    z = z * 1.0_Qs3_12;
    
    std::cout << "z: " << z.integer() << ' ' << z.fractional()/(double)FP::one << '\n';
    
    auto y = x * c;

    std::cout << "y: " << y.integer() << ' ' << y.fractional()/(double)FP::one << '\n';
  
    y += FP{0.5};
    std::cout << "y: " << y.integer() << ' ' << y.fractional()/(double)FP::one << '\n';
    
    
    std::cout << std::setbase(16) << c.raw() << '\n';
}
