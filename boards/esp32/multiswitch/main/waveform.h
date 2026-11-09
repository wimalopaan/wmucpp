#pragma once

#include <cstdint>

#include "board.h"

namespace Multiswitch {
    struct Waveform {
        uint8_t duties[8]{};
        uint32_t pins[8]{};
        uint32_t period = 1'000;

        constexpr Waveform() {
            for (unsigned i = 0; i < 8; ++i) pins[i] = uint32_t{1} << Board::outputs[i];
        }

        struct Edge {
            uint32_t high;
            uint8_t nextPhase;
        };

        [[gnu::always_inline]] inline Edge sample(const uint32_t elapsed, const uint8_t phase) const {
            Edge edge{0, 100};
            for (unsigned i = 0; i < 8; ++i) {
                const uint8_t duty = duties[i];
                const uint32_t fallingEdge = uint32_t{duty} * period / 100;
                if (duty == 100 || fallingEdge > elapsed) edge.high |= pins[i];
                if (duty > phase && duty < 100 && fallingEdge > elapsed && duty < edge.nextPhase) {
                    edge.nextPhase = duty;
                }
            }
            return edge;
        }
    };
}
