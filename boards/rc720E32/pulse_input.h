#pragma once

#include <cstdint>
#include <chrono>

#include "mcu/alternate.h"
#include "output.h"
#include "util.h"

using namespace std::literals::chrono_literals;

namespace Pulse {
    template<auto N, typename Config, typename MCU = DefaultMcu>
    struct CppmIn {
        using pin = Config::pin;
        using clock = Config::clock;
        using systemTimer = Config::timer;
        using tp = Config::tp;

        static inline void init() {
        }
        static inline void reset() {
        }
        static inline void update() {

        }
        static inline void periodic() {

        }
        static inline void ratePeriodic() {

        }
    };
}
