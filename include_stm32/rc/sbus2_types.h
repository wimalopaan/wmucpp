#pragma once

#pragma once

#include <cstdint>
#include <cstddef>

#include "etl/ranged.h"

namespace RC::Protokoll::SBus2 {
    using namespace etl::literals;
    using namespace std::literals::chrono_literals;
    static inline constexpr uint32_t baud = 100'000;
    static inline constexpr uint8_t start_byte = 0x0f;
    static inline constexpr uint8_t end_byte = 0x00;

    static inline constexpr uint8_t flagsIndex = 23;
    static inline constexpr uint8_t endIndex = 24;

    static inline constexpr uint8_t ch17 = 0x01;
    static inline constexpr uint8_t ch18 = 0x02;
    static inline constexpr uint8_t frameLost = 0x04;
    static inline constexpr uint8_t failSafe = 0x08;

    inline static constexpr uint16_t sbus_min = 172;
    inline static constexpr uint16_t sbus_max = 1811;

    inline static constexpr uint16_t sbus_mid = (sbus_max + sbus_min) / 2;

    using value_type = etl::ranged<sbus_min, sbus_max>;
    using index_type = etl::ranged<0, 15>;
}
