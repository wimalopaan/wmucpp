#pragma once

#include <cstdint>
#include <mcu/internals/systemclock.h>

namespace Util {
    template<size_t N, size_t Rows, size_t Columns, typename E = std::byte>
    struct FramBuffer {
        using value_type = E;
        static inline constexpr size_t size() {
            return (Rows / 8) * Columns;
        }
        static inline constexpr E& operator[](const size_t index) {
            
        }
    private:
        static inline std::array<E, size()> mData;
    };
}
