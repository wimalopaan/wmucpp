#pragma once

#include <type_traits>
#include <array>

#include "concepts.h"
#include "meta.h"
#include "algorithm.h"

namespace etl {
    template<etl::Concepts::Unsigned T = size_t>
    struct Straddle {
        constexpr Straddle(const T& b, const T& l) : beginPosition{b}, length{l} {}
        constexpr bool overlap(const Straddle& o) const {
            return (o.includes(beginPosition) || o.includes(beginPosition + length - 1));
        }
        constexpr bool includes(const T& p) const {
            return ((p >= beginPosition) && (p < (beginPosition + length)));
        }
    private:
        const T beginPosition;
        const T length;
    };
    
    
    template<uint8_t N>
    using BitNumber = std::integral_constant<uint8_t, N>;
    template<uint8_t N>
    using BitPosition = std::integral_constant<uint8_t, N>;
    
    template<typename Name, etl::Concepts::NamedConstant Number, etl::Concepts::NamedConstant Position>
    struct Bits {
        using name_type = Name;
        using number_type = Number;
        using position_type = Position;
        inline static constexpr std::byte mask{(1 << Number::value) - 1};
        inline static constexpr uint8_t position{Position::value};
        inline static constexpr uint8_t number = Number::value;
    };

    template<typename BitList, typename Underlying = std::byte> struct BitField;
    
    template<typename... BB, typename Underlying>
    struct BitField<Meta::List<BB...>, Underlying> {
        inline static constexpr auto totalNumberOfBits{(BB::number + ...)};
        static_assert(totalNumberOfBits <= (sizeof(Underlying) * 8));
        
        inline static constexpr bool overlap = []{
            std::array<Straddle<>, sizeof...(BB)> straddles{Straddle<>{BB::position, BB::number}...};
            for(size_t i{}; i < straddles.size(); ++i) {
                for(size_t n{i + 1}; n < straddles.size(); ++n) {
                    if (straddles[i].overlap(straddles[n])) {
                        return true;
                    }       
                }
            }
            return false;
        }();
        static_assert(!overlap, "Bits must not overlap");

        template<auto N = 0>        
        inline constexpr std::byte byte() const {
            return etl::nth_byte<N>(data);
        }
        
        template<typename B>
        inline constexpr void set(std::byte b) {
            static_assert(Meta::contains_v<Meta::List<BB...>, B>);
            data = (data & ~(B::mask << B::position)) | ((b & B::mask) << B::position);
        }
        
    private:
        Underlying data;
    };
}
