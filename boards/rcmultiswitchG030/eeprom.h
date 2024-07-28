#pragma once

#define 	FLASH_KEYR_KEY1   ((uint32_t)0x45670123)
#define 	FLASH_KEYR_KEY2   ((uint32_t)0xcdef89ab)

extern char const _flash_start;
extern char const _eeprom_start;

template<typename T>
bool savecfg(const T& eeprom, const T& flash) {
    __disable_irq();
    FLASH->KEYR = FLASH_KEYR_KEY1;
    FLASH->KEYR = FLASH_KEYR_KEY2;
    FLASH->SR = -1; // Clear errors

    FLASH->CR = FLASH_CR_PER;
#ifdef STM32G0
    FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT | (((uint32_t)(&_eeprom_start - &_flash_start) >> 11) << FLASH_CR_PNB_Pos); // Erase page
#else
    FLASH->AR = (uint32_t)_cfg;
    FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT; // Erase page
#endif
    while (FLASH->SR & FLASH_SR_BSY1);

    FLASH->CR = FLASH_CR_PG;
#ifdef STM32G0
    using value_type = uint32_t;
#else
    using value_type = uint16_t;
#endif
    value_type* dst = (value_type*)&flash;
    // value_type* dst = (value_type*)&_eeprom_start; // why not???
    value_type* src = (value_type*)&eeprom;
    value_type* end = (value_type*)(((char*)&eeprom) + sizeof(T));
    while (src < end) { // Write data
        *dst++ = *src++;
#ifdef STM32G0
        *dst++ = *src++;
#endif
        while (FLASH->SR & FLASH_SR_BSY1);
    }
    FLASH->CR = FLASH_CR_LOCK;
    __enable_irq();
#ifdef STM32G0
    if (FLASH->SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR)) return false;
#else
    if (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) return 0;
#endif

    return (memcmp(&_eeprom_start, &eeprom, sizeof(T)) == 0);
}
