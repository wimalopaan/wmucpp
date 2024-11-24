#pragma once

#include <cstdint>

template<typename R>
struct RessourceCount {
    static inline void acquire(const auto f) {
        if (mCounter == 0) {
            f();
        }
        ++mCounter;
    }
    static inline void release(const auto f) {
        if (mCounter > 0) {
            if (--mCounter == 0) {
                f();
            }
        }
    }
    private:
    static inline uint8_t mCounter = 0;
};
