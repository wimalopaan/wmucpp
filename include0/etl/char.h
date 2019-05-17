
#pragma once

#include <cstdint>

namespace etl {
    enum class Char : uint8_t {};
    
    constexpr inline bool isDigit(Char c) {
        return ((static_cast<uint8_t>(c) >= '0') && (static_cast<uint8_t>(c) <= '9'));
    }

    constexpr inline uint8_t asDigit(Char c) {
        return (static_cast<uint8_t>(c) - '0');
    }
    
}
