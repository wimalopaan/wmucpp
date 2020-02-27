#pragma once

#include <type_traits>
#include "char.h"

namespace etl {
    namespace detail {
        constexpr inline uint8_t to_int(const char c) {
            uint8_t result{};
            
            if (c >= 'A' && c <= 'F') {
                result = static_cast<uint8_t >(c) - static_cast<uint8_t >('A') + 10;
            }
            else if (c >= 'a' && c <= 'f') {
                result = static_cast<uint8_t >(c) - static_cast<uint8_t >('a') + 10;
            }
            else {
                result = static_cast<uint8_t >(c) - static_cast<uint8_t >('0');
            }
            return result;
        }
        
        template<size_t N>
        consteval inline unsigned long long parse(const char (&arr)[N]) {
            unsigned long long base{10};
            size_t offset{0};
            
            if (N > 2) {
                bool starts_with_zero{arr[0] == '0'};
                bool is_hex{starts_with_zero && (arr[1] == 'x')};
                bool is_binary{starts_with_zero && (arr[1] == 'b')};
                
                if (is_hex) {
                    //0xDEADBEEF (hexadecimal)
                    base = 16;
                    offset = 2;
                }
                else if (is_binary) {
                    //0b101011101 (binary)
                    base = 2;
                    offset = 2;
                }
                else if (starts_with_zero) {
                    //012345 (octal)
                    base = 8;
                    offset = 1;
                }
            }
            
            unsigned long long number{0};
            long long multiplier{1};
            
            for (size_t i{0}; i < N - offset; ++i) {
                const char c = arr[N - 1 - i];
                if (c != '\'') { // skip digit separators
                    number += detail::to_int(c) * multiplier;
                    multiplier *= base;
                }
            }
            return number;
        }
    }

    namespace literals {
        template <char... CC>
        consteval auto operator"" _c() {
            return std::integral_constant<size_t, detail::parse<sizeof...(CC)>({CC...})>{};
        }
    }
}

namespace std {
    template<size_t N>
    constexpr inline auto operator-(std::integral_constant<size_t, N>) {
        return std::integral_constant<std::make_signed_t<size_t>, -N>{};    
    }
}



