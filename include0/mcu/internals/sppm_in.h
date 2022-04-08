#pragma once

#include <std/chrono>
#include <etl/rational.h>

#include "../common/isr.h"
#include "../common/pwm.h"
#include "../common/concepts.h"

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;
    
    namespace Ppm {
        template<auto TimerNumber, typename MCU = DefaultMcuType>
        struct SinglePpmIn;
    }
}
