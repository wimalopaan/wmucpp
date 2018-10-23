#pragma once

#include <cstdint>

#include "config.h"
#include "mcu/avr8.h"

#include <avr/sleep.h>

namespace AVR {
    template<typename M = DefaultMcuType>
    class Sleep {
        inline static constexpr auto cr = AVR::getBaseAddr<AVR::ATTiny85::MCUControl>;
    public:
        struct PowerDown;

        template<typename T>
        inline static void init() {
            if constexpr(std::is_same_v<T, PowerDown>) {
                cr()->value.set<AVR::ATTiny85::MCUControl::Bits::sleepEnable | AVR::ATTiny85::MCUControl::Bits::sleepMode1>();
            }
        }
        inline static void down() {
            if constexpr(MCU::is_avr_v<M>) {
                __asm__ __volatile__ ( "sleep" "\n\t" :: ); 
            }
            else {
                static_assert(std::false_v<M>, "MCU not supported");
            }
        }
    private:
    };
}
