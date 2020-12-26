#pragma once

#include <cstdint>
#include <std/utility>

namespace AVR {
    namespace SeriesDa {
        struct SigRow final {
            DataRegister<SigRow, ReadOnly, std::byte> deviceId0;
            DataRegister<SigRow, ReadOnly, std::byte> deviceId1;
            DataRegister<SigRow, ReadOnly, std::byte> deviceId2;
            
            const volatile std::byte reserved;

            DataRegister<SigRow, ReadOnly, uint16_t> tempSense0;
            DataRegister<SigRow, ReadOnly, uint16_t> tempSense1;

            const volatile std::byte reserved2[0x0f - 0x08 + 1];
            
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
            DataRegister<SigRow, ReadOnly, std::byte> serNuma;
            DataRegister<SigRow, ReadOnly, std::byte> serNumb;
            DataRegister<SigRow, ReadOnly, std::byte> serNumc;
            DataRegister<SigRow, ReadOnly, std::byte> serNumd;
            DataRegister<SigRow, ReadOnly, std::byte> serNume;
            DataRegister<SigRow, ReadOnly, std::byte> serNumf;
            
            static inline constexpr uintptr_t address = 0x1100;
        
        };
        
        static_assert(sizeof(SigRow) == 0x20);
    }
}
