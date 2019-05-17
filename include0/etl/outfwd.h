#pragma once

#include "concepts.h"

namespace etl {
    template<etl::Concepts::Stream Stream, typename... TT>
    inline constexpr void out(const TT&... v __attribute__((unused)));

    template<etl::Concepts::Stream Stream, typename... TT>
    inline constexpr void outl(const TT&... v __attribute__((unused)));       
}
