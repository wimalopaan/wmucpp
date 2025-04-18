#pragma once

#include <cstdint>

namespace etl {
    template<auto N = 4096>
    static inline auto normalize(auto v) {
        while (v >= N) {
            v -= N;
        }
        while (v < 0) {
            v += N;
        }
        return v;
    }
}

