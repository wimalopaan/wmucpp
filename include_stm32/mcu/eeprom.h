#pragma once

#include <cstdint>

#include "atomic.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#define 	FLASH_KEYR_KEY1   ((uint32_t)0x45670123)
#define 	FLASH_KEYR_KEY2   ((uint32_t)0xcdef89ab)

extern char const _flash_start;
extern char const _eeprom_start;

namespace Mcu::Stm32 {
    template<typename T>
    std::pair<bool, uint8_t> savecfg(const T& eeprom, const T& flash) {
        uint8_t eeprom_status = 0;

        Mcu::Arm::Atomic::DisableInterruptsRestore _di;

        FLASH->KEYR = FLASH_KEYR_KEY1;
        FLASH->KEYR = FLASH_KEYR_KEY2;

        eeprom_status = FLASH->SR & 0xff;
        if (FLASH->SR & 0xff) {
            return {false, eeprom_status};
        }

        FLASH->SR = -1; // Clear errors

#if defined(STM32G0)
        FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT | (((uint32_t)(&_eeprom_start - &_flash_start) >> 11) << FLASH_CR_PNB_Pos); // Erase page
#elif defined(STM32G4)
        FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT | (((uint32_t)(&_eeprom_start - &_flash_start) >> 11) << FLASH_CR_PNB_Pos); // Erase page
        // FLASH->CR |= FLASH_CR_BKER;
#else
#error "No MCU defined"
#endif
#if defined(STM32G0)
        while (FLASH->SR & FLASH_SR_BSY1);
#elif defined(STM32G4)
        while (FLASH->SR & FLASH_SR_BSY);
#else
#error "No MCU defined"
#endif

        eeprom_status = FLASH->SR & 0xff;
        if (FLASH->SR & 0xff) {
            return {false, eeprom_status};
        }

        FLASH->SR = -1; // Clear errors
        FLASH->CR = FLASH_CR_PG;
#if defined(STM32G0) || defined(STM32G4)
        using value_type = uint32_t;
#else
        using value_type = uint16_t;
#endif
        volatile value_type* dst = (value_type*)&flash;
        // value_type* dst = (value_type*)&_eeprom_start; // why not???
        volatile value_type* src = (value_type*)&eeprom;
        value_type* end = (value_type*)(((char*)&eeprom) + sizeof(T));

        while (src < end) { // Write data
            *dst++ = *src++;
#if defined(STM32G0) || defined(STM32G4)
            *dst++ = *src++;
#endif
#if defined(STM32G0)
            while (FLASH->SR & FLASH_SR_BSY1);
#elif defined(STM32G4)
            while (FLASH->SR & FLASH_SR_BSY);
#else
#error "No MCU defined"
#endif
            if (FLASH->SR & 0xff) {
                break;
            }
        }
        eeprom_status = FLASH->SR & 0xff;
        FLASH->CR = FLASH_CR_LOCK; // and clears all other bits (e.g. PG bit)

        if (FLASH->SR & 0xff) {
            return {false, eeprom_status};
        }
        return {(memcmp(&flash, &eeprom, sizeof(T)) == 0), eeprom_status};
    }
}
#pragma GCC diagnostic pop
