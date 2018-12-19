#pragma once

#include <cstdint>
#include <type_traits>

#include "type_traits.h"

namespace etl {
    using namespace std;
    
    enum class Char : uint8_t {};
    
    template<bool V>
    struct NamedFlag : integral_constant<bool, V> {};
    
    template<auto c>
    struct NamedConstant : integral_constant<decltype(c), c> {};
    
    template<auto Bits>
    class bitsN_t {
    public:
        inline static constexpr auto size = Bits;
        typedef typeForBits_t<Bits> value_type;
        inline static constexpr value_type mask = ((1 << Bits) - 1);
        constexpr bitsN_t(const volatile bitsN_t& o) : mValue{o.mValue} {}
        constexpr bitsN_t() = default;
        constexpr explicit bitsN_t(value_type v) : mValue(v & mask) {}
        constexpr explicit bitsN_t(std::byte v) : mValue(std::to_integer<value_type>(v) & mask) {}
        constexpr explicit operator value_type() const {
            return mValue;
        }
    private:
        value_type mValue{};
    };
    
    template<auto Bits>
    class uintN_t {
    public:
        typedef typeForBits_t<Bits> value_type;
        inline static constexpr value_type mask = ((1 << Bits) - 1);
        explicit uintN_t(value_type v = 0) : mValue(v & mask) {}
        constexpr operator value_type() const {
            return mValue;
        }
        constexpr uintN_t& operator++() {
            ++mValue;
            mValue &= mask;
            return *this;
        }
    private:
        value_type mValue{};
    };
    
}
