#include <stdint.h>
#include <stdbool.h>

typedef union {
  uint8_t rawByte;
  struct {
    bool date : 1;
    bool warning : 1;
    bool lowBattery : 1;
    bool temp : 1;
    bool pressure : 1;
  };
} un_displaySymbols;

typedef enum {
  DISPLAY_SYMBOL_DATE = 1 << 0,
  DISPLAY_SYMBOL_WARNING = 1 << 1,
  DISPLAY_SYMBOL_LOW_BATTERY = 1 << 2,
  DISPLAY_SYMBOL_TEMP = 1 << 3,
  DISPLAY_SYMBOL_PRESSURE = 1 << 4,
} en_displaySymbols;

void Display_SetSymbol(en_displaySymbols, bool) {}

uint8_t v;

int main() {
//    un_displaySymbols v;
//    v.rawByte = 1;
    
    Display_SetSymbol(DISPLAY_SYMBOL_DATE, v & DISPLAY_SYMBOL_DATE);
    Display_SetSymbol(DISPLAY_SYMBOL_WARNING, v & DISPLAY_SYMBOL_WARNING);
}

