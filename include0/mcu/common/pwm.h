#pragma once

#include "../../config.h"
#include <cstdint>
#include <external/units/physical.h>

#include "timer.h"

namespace AVR {
    using namespace Project;
    
    namespace PWM {
        
        namespace Util {
            using namespace AVR::Util::Timer;
            using namespace std::literals::chrono;
            using namespace External::Units::literals;
            
            // returns the highest prescaler for a pwm-frequency above ftimer
            // returns the lowest frequency only with prescaler (no ocr) above ftimer
            template<auto TimerNumber, typename MCU = DefaultMcuType>
            constexpr uint16_t prescalerForAbove(const hertz& ftimer) {
                using pBits = prescaler_bits_t<TimerNumber>;
                auto prescalers = prescalerValues(pBits::values);
                for(const auto& p : etl::sort(prescalers, std::greater<uint16_t>())) { // absteigend
                    if (p > 0) {
                        auto f = Config::fMcu / p;
                        if (f >= ftimer) {
                            return p;
                        }
                    }
                }
                return 0;
            }
            
        }
    }
}
