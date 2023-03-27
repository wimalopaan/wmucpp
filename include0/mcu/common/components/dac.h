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
    namespace SeriesDb {
        struct Dac {
            enum class CtrlA_t : uint8_t {
                standby = DAC_RUNSTDBY_bm,
                enable  = DAC_ENABLE_bm,
                outen   = DAC_OUTEN_bm,
            };  
            ControlRegister<Dac, CtrlA_t> ctrla;
            const volatile std::byte reserved;
            DataRegister<Dac, ReadWrite, uint16_t> data;

            template<int N> struct Address;
        };
    }
}
