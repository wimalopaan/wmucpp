#pragma once

#include "types.h"


namespace etl {
    template<auto T>
    struct static_print_v {
#ifndef NDEBUG
        std::integral_constant<uint16_t, T> v;
        using type = typename decltype(v)::_;
#endif
    };
    template<typename T>
    struct static_print_t {
#ifndef NDEBUG
        using type = T::_;
#endif
    };
}
