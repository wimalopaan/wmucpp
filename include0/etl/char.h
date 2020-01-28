
#pragma once

#include <cstdint>

namespace etl {
    enum class Char : uint8_t {};
    
    constexpr inline bool isDigit(Char c) {
        return ((static_cast<uint8_t>(c) >= '0') && (static_cast<uint8_t>(c) <= '9'));
    }
    constexpr inline bool isUpper(Char c) {
        return ((static_cast<uint8_t>(c) >= 'A') && (static_cast<uint8_t>(c) <= 'Z'));
    }
    constexpr inline bool isLower(Char c) {
        return ((static_cast<uint8_t>(c) >= 'a') && (static_cast<uint8_t>(c) <= 'z'));
    }

    constexpr inline uint8_t asDigit(Char c) {
        return (static_cast<uint8_t>(c) - '0');
    }

    constexpr inline Char toLower(Char c) {
        if (isUpper(c)) {
            return Char(uint8_t(c) + 0x20u);
        }
        return c;
    }    
}
