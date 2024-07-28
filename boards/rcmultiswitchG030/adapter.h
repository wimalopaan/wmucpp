#pragma once

#include <cstdint>
#include <type_traits>
#include <output.h>
#include <mcu/alternate.h>

#include "tick.h"

namespace Local {
    template<typename Pwm, auto Channel, bool Negativ = false, bool HasOutput = true, typename Debug = void>
    struct PwmAdapter {
        static inline constexpr bool hasOutput = HasOutput;
        static inline constexpr uint8_t channel = Channel;
        using pwm = Pwm;
        using component_t = Pwm::component_t;
        using polarity_t = std::conditional_t<Negativ, Mcu::Stm::AlternateFunctions::Negativ, Mcu::Stm::AlternateFunctions::Positiv>;

        static inline void duty(const uint8_t d) {
            const uint16_t dv = (Pwm::period * (uint32_t)d) / 100;
            if constexpr(Channel == 1) {
                IO::outl<Debug>("duty1: ", dv);
                Pwm::duty1(dv);
            }
            else if constexpr(Channel == 2){
                IO::outl<Debug>("duty2: ", dv);
                Pwm::duty2(dv);
            }
            else if constexpr(Channel == 3){
                IO::outl<Debug>("duty3: ", dv);
                Pwm::duty3(dv);
            }
            else if constexpr(Channel == 4){
                IO::outl<Debug>("duty4: ", dv);
                Pwm::duty4(dv);
            }
            else {
                static_assert(false);
            }
        }
    };

}

