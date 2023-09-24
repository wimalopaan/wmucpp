/*
  Devil-Elec
  Nano Development Board v1.0
  IDE 1.8.19
  Arduino Nano Every
  19.09.2023 
*/

#include "AVRxDB_Pin.h"
#include "Encoder.h"

Encoder <pinPA5, pinPA6, 2, -5, 5>          Encoder1;   

#define LEDS_DDR    PORTD_DIR
#define LEDS        PORTD_OUT

void setup (void)
{
    LEDS_DDR = 0xff;
    Encoder1.init();  
    Encoder1.enablePullup(); 
}

void updateEncoder (void);

void loop (void)
{
    updateEncoder();
}

void updateEncoder (void)
{   
    Encoder1.encode();        // Daten aktualisieren
//    LEDS   = Encoder1.getRelCounter(); 
    LEDS   = Encoder1.getAbsCounter(); 
}

int main() {
    setup();
    while(true) {
        loop();
    }
}
