#pragma once

#include <cstdint>
#include <optional>
#include <type_traits>
#include <limits>

#include "traits.h"

namespace etl {
    using namespace std;

    template<typename T, uint16_t Size = 32>
    class CircularBuffer final {
    };
}
