#pragma once

#include <cstdint>

template<typename Config>
struct InputMapper {
    using stream1 = Config::stream1;
    using stream2 = Config::stream2;
    using stream3 = Config::stream3;

    static inline constexpr uint16_t mid = stream1::mid;
    static inline constexpr uint16_t amp = stream1::amp;

    static inline uint16_t value(const uint8_t ch) {
        switch(mStream) {
        case 0: // Main CRSF
            return stream1::value(ch);
            break;
        case 1: // Relay-Port
            return stream2::value(ch);
            break;
        case 2: // Aux-Port
            if constexpr(!std::is_same_v<stream3, void>) {
                return stream3::value(ch);
            }
            else {
                return mid;
            }
            break;
        default:
            return mid;
            break;
        }
    }
    static inline void stream(const uint8_t s) {
        mStream = s;
    }
    private:
    static inline uint8_t mStream = 0;
};
