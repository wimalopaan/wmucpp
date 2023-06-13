#pragma once

using namespace Units::literals;

namespace Arm {
    template<typename Clock, Units::megahertz f = 2_MHz>
    struct Trace {
        static inline void init() {
            TPI->ACPR = Clock::config::f / f - 1;
        }
        static inline void put(const char c) {
            ITM_SendChar(c);
        }
    };
}
