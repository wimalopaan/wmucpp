#pragma once

#include "../../config.h"
#include <cstdint>
#include <external/units/physical.h>

namespace AVR {
    using namespace External::Units;
    template<typename T>
    struct TimerSetupData final {
        const uint16_t prescaler = 0;
        const T ocr = 0;
        const hertz f{0};
        const bool isExact = false;
        explicit constexpr operator bool() const {
            return (prescaler > 0) && (ocr > 0);
        }
    };
    
}
