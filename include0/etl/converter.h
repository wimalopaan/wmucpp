#pragma once

#include <cstdint>
#include <algorithm>
#include <array>

#include "char.h"

namespace etl {
    
    template<typename F>
    struct StringConverter;
    
    template<uint8_t N>
    struct StringConverter<FixedPoint<int16_t, N>> {
        template<uint8_t L>
        inline static FixedPoint<int16_t, N> parse(const StringBuffer<L>& str) {
            int16_t value = 0;
            int8_t decimals = -1;
            bool negative = false;
            
            uint8_t index = 0;
            
            if (str[0] == etl::Char{'-'}) {
                negative = true;
                ++index;
            }
            
            for(; index < L; ++index) {
                if (str[index] == etl::Char{'.'}) {
                    decimals = 0;
                }
                else if (isDigit(str[index])) {
                    value *= 10;
                    value += asDigit(str[index]);
                    if (decimals >= 0) {
                        ++decimals;
                    }
                }
                else if (str[index] == etl::Char{'\0'}) {
                    break;
                }
            }
            value <<= N;
            while(decimals > 0) {
                value /= 10;
                --decimals;
            }
            if (negative) {
                value = -value;
            }
            return FixedPoint<int16_t, N>::fromRaw(value);
        }
    };
}
