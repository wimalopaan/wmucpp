#pragma once

#include <cstdint>
#include <cstddef>

#include "external/units/physical.h"

namespace Project {
    using megahertz = External::Units::megahertz;
    using hertz     = External::Units::hertz;
    
    struct Config final {
        Config() = delete; // one should use a similar copy in own project
    
        inline static constexpr megahertz fMcuMhz {F_CPU / 1000000};
        inline static constexpr hertz fMcu{F_CPU};
    
        static_assert(fMcuMhz.value <= 32, "F_CPU too high");
        static_assert(fMcuMhz.value >=  1, "F_CPU too low");

        inline static constexpr hertz fRtc{32768};
        
        static_assert(fRtc.value <= 32768, "F_RTC too high");
        static_assert(fRtc.value >=  1024, "F_RTC too low");
    };

    enum class variants_t : uint8_t {A, B, C, D, E, F, H, S};
    
    template<variants_t V, typename SubVariant = void>
    struct Variant : std::integral_constant<variants_t, V> {
        using sub_type = SubVariant;
    };
}
