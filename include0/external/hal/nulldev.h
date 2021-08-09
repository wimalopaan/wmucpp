#pragma once

namespace External::Hal {
    template<typename MCU = DefaultMcuType>
    struct NullDevice {
        static inline constexpr void init() {}        
    };
}
