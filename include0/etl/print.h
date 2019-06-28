#pragma once

#include "types.h"

namespace etl {
    template<auto T>
    struct static_print_v {
        using type = std::integral_constant<uint16_t, uint16_t(T)>::_;
    };

    template<typename T>
    struct static_print_t {
        using type = T::_;
    };
}

