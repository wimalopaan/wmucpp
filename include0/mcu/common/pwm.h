#pragma once

#include "../../config.h"
#include <cstdint>
#include <external/units/physical.h>

namespace AVR {
    using namespace Project;
    
    namespace PWM::Util {
        using namespace AVR::Util::Timer;
        using namespace std::literals::chrono;
        using namespace External::Units::literals;
    }
}
