#pragma once

#include <cstdint>
#include <std/utility>

namespace AVR {
    namespace Series1 {
        struct Dac {
            enum class CtrlA_t : uint8_t {
                standby = DAC_RUNSTDBY_bm,
                enable  = DAC_ENABLE_bm,
                outen   = DAC_OUTEN_bm,
            };  
            ControlRegister<Dac, CtrlA_t> ctrla;
            DataRegister<Dac, ReadWrite, std::byte> data;

            template<int N> struct Address;
        };
    }
}
