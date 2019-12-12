#pragma once

#include <type_traits>

#include "concepts.h"
#include "meta.h"

namespace etl {
    template<etl::Concepts::Unsigned T>
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
            using straddle_type = etl::typeForBits_t<totalNumberOfBits>;
            std::array<Straddle<straddle_type>, sizeof...(BB)> straddles{Straddle<straddle_type>{BB::position, BB::number}...};
            
            for(decltype(straddles.size()) i{0}; i < straddles.size(); ++i) {
                for(decltype(straddles.size()) n{i + 1}; n < straddles.size(); ++n) {
                    if (straddles[i].overlap(straddles[n])) {
                        return true;
                    }       
                }
            }
            return false;
        }();
        static_assert(!overlap);

        template<auto N>        
        inline constexpr std::byte byte() const {
            return etl::nth_byte<N>(data);
        }
    private:
        Underlying data;
    };
    
}
