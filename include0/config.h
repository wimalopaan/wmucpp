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
    
        static_assert(fMcuMhz.value <= 20, "F_CPU too high");
        static_assert(fMcuMhz.value >=  1, "F_CPU too low");

        inline static constexpr hertz fRtc{32768};
        
        static_assert(fRtc.value <= 32768, "F_RTC too high");
        static_assert(fRtc.value >=  1024, "F_RTC too low");
        
        
    //    struct Timer {
    //        inline static constexpr uint8_t NumberOfTimers = 8;
    //        inline static constexpr std::hertz frequency = 100_Hz;
    //        inline static constexpr std::milliseconds resolution = std::duration_cast<std::milliseconds>(1 / frequency);
    //    };
    //    struct EventManager {
    //        inline static constexpr uint8_t EventQueueLength = 32;
    //    };
    //    struct Usart {
    //        inline static constexpr uint8_t SendQueueLength = 64;
    //        inline static constexpr uint8_t RecvQueueLength = 0;
    //        inline static constexpr bool  useEvents = true;
    //    };
    //    struct SoftSpiMaster {
    //        inline static constexpr std::microseconds pulseDelay = 1_us;
    //    };
    //    struct Button {
    //        inline static constexpr uint8_t buttonTicksForPressed = 50_ms * Timer::frequency;
    //    };
    //    inline static constexpr std::microseconds zeroMicroSeconds{0};
    //    inline static constexpr bool ensureTerminalOutput = true;
    //    inline static constexpr bool disableCout = false;
    };
    
}
