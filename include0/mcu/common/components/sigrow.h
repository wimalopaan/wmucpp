#pragma once

#include <cstdint>
#include <std/utility>

namespace AVR {
    namespace Series0 {
        struct SigRow final {
            DataRegister<SigRow, ReadOnly, std::byte> deviceId0;
            DataRegister<SigRow, ReadOnly, std::byte> deviceId1;
            DataRegister<SigRow, ReadOnly, std::byte> deviceId2;
            
            DataRegister<SigRow, ReadOnly, std::byte> serNum0;
            DataRegister<SigRow, ReadOnly, std::byte> serNum1;
            DataRegister<SigRow, ReadOnly, std::byte> serNum2;
            DataRegister<SigRow, ReadOnly, std::byte> serNum3;
            DataRegister<SigRow, ReadOnly, std::byte> serNum4;
            DataRegister<SigRow, ReadOnly, std::byte> serNum5;
            DataRegister<SigRow, ReadOnly, std::byte> serNum6;
            DataRegister<SigRow, ReadOnly, std::byte> serNum7;
            DataRegister<SigRow, ReadOnly, std::byte> serNum8;
            DataRegister<SigRow, ReadOnly, std::byte> serNum9;
            
            const volatile std::byte reserved[0x1f - 0x0d + 1];
            
            DataRegister<SigRow, ReadOnly, std::byte> tempSense0;
            DataRegister<SigRow, ReadOnly, std::byte> tempSense1;
            
            DataRegister<SigRow, ReadOnly, std::byte> osc16err3V;
            DataRegister<SigRow, ReadOnly, std::byte> osc16err5V;
            
            DataRegister<SigRow, ReadOnly, std::byte> osc20err3V;
            DataRegister<SigRow, ReadOnly, std::byte> osc20err5V;

        
            static inline constexpr uintptr_t address = 0x1100;
        
        };
        
        static_assert(sizeof(SigRow) == 0x26);
    }
}
