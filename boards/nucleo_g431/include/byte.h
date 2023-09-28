#pragma once

#include <cstddef>

namespace etl::literals {
    constexpr std::byte operator ""_B(unsigned long long v) {
        return std::byte{static_cast<uint8_t>(v)};
    }
    constexpr std::byte operator ""_B(char v) {
        return std::byte{v};
    }
}

