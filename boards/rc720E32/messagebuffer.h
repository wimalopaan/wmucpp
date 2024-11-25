#pragma once

#include <cstdint>

#include "tick.h"
#include "etl/fifo.h"

template<typename Out, typename Timer>
struct MessageBuffer {
    struct Entry {
        using value_type = std::byte;
        static inline constexpr uint8_t size = 64;
        void operator=(const std::span<volatile uint8_t>& s) {
            length = std::min((uint8_t)s.size(), size);
            auto ptr = &s[0];
            for(auto& v: message) {
                v = (value_type)*ptr++;
            }
        }
        void operator=(const std::span<volatile std::byte>& s) {
            length = std::min((uint8_t)s.size(), size);
            auto ptr = &s[0];
            for(auto& v: message) {
                v = *ptr++;
            }
        }
        uint8_t length;
        std::array<value_type, size> message;
    };
    static inline void enqueue(const auto& c) {
        mFifo.emplace_back(c);
    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
        if (Out::isIdle()) {
            if (!mFifo.empty()) {
                volatile std::byte* data = Out::outputBuffer();
                const auto& m = mFifo.front();
                for(uint8_t i = 0; (i < m.length) && (i < Out::size); ++i) {
                    data[i] = m.message[i];
                }
                Out::startSend(m.length);
                mFifo.pop_front();
            }
        }
    }
    private:
    static inline etl::FiFo<Entry> mFifo;
};
