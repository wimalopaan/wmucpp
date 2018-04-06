#pragma once

namespace Util {
    
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
            
            if (str[0] == '-') {
                negative = true;
                ++index;
            }
            
            for(; index < L; ++index) {
                if (str[index] == '.') {
                    decimals = 0;
                }
                else if ((str[index] >= '0') && (str[index] <= '9')) {
                    value *= 10;
                    value += (str[index] - '0');
                    if (decimals >= 0) {
                        ++decimals;
                    }
                }
                else if (str[index] == '\0') {
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
