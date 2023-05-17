#include <cstdint>

typedef union
{
  uint8_t rawByte;
  struct
  {
    uint8_t date : 1;
    uint8_t warning : 1;
    uint8_t lowBattery : 1;
    uint8_t temp : 1;
    uint8_t pressure : 1;
  };
}un_displaySymbols;

typedef enum
{
  DISPLAY_SYMBOL_DATE,
  DISPLAY_SYMBOL_WARNING,
  DISPLAY_SYMBOL_LOW_BATTERY,
  DISPLAY_SYMBOL_TEMP,
  DISPLAY_SYMBOL_PRESSURE,
} en_displaySymbols;

void Display_SetSymbol(en_displaySymbols, bool) {}

un_displaySymbols v;

int main() {
    v.rawByte = 1;
    Display_SetSymbol(DISPLAY_SYMBOL_DATE, v.date);
}
