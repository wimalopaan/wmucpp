#pragma once

#include <cstdint>

#include "tick.h"
#include "etl/fifo.h"

template<typename Out, typename Timer>
struct MessageBuffer {

    static inline void enqueue(const auto& c) {
        mFifo.emplace_back(c);
    }

    static inline void periodic() {
    }
    static inline void ratePeriodic() {
        if (Out::isIdle()) {

        }
    }

    private:
    static inline etl::FiFo<std::array<uint8_t, 64>> mFifo;
};
