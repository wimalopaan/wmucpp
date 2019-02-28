#pragma once

#include "../../config.h"
#include <cstdint>
#include <external/units/physical.h>

#include "timer.h"

namespace AVR {
    using namespace Project;
    
    namespace PWM::Util {
        using namespace AVR::Util::Timer;
        using namespace std::literals::chrono;
        using namespace External::Units::literals;
        
        template<auto TimerNumber, typename MCU = DefaultMcuType>
        constexpr uint16_t prescalerForAbove(const hertz& ftimer) {
            using mcu_timer_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_type;
//            using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;  
            
            using pBits = typename mcu_timer_type::template PrescalerBits<TimerNumber>;
            auto p = prescalerValues(pBits::values);
            for(const auto& p : etl::sort(p, std::less<uint16_t>())) {
                if (p > 0) {
                    auto f = Config::fMcu / p;
                    if (f >= ftimer) {
                        return p;
                    }
                }
            }
            return 0;
        }

        template<auto TimerNumber, typename MCU = DefaultMcuType>
        constexpr auto calculate(const hertz& ftimer) {
            using mcu_timer_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_type;
            using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;  
            
            using pBits = typename mcu_timer_type::template PrescalerBits<TimerNumber>;
            auto p = prescalerValues(pBits::values);
            
            for(const auto& p : etl::sort(p)) { // aufsteigend
                if (p > 0) {
                    const auto tv = (Config::fMcu / ftimer) / p;
                    if ((tv > 0) && (tv < std::numeric_limits<value_type>::max())) {
                        const bool exact = ((Config::fMcu.value / p) % tv) == 0;
                        return AVR::TimerSetupData<value_type>{p, static_cast<value_type>(tv), Config::fMcu / tv / uint32_t(p), exact};
                    }
                }
            }
            return AVR::TimerSetupData<value_type>{};
        }
        
        
    }
}
