#include <avr/io.h>
#include <avr/interrupt.h>

namespace {
#if 1
    volatile uint8_t nextPulseState[256]{};
    volatile uint8_t bufferPointer = 0;
#else 
    uint8_t nextPulseState[256]{};
    uint8_t bufferPointer = 0;
#endif
    uint8_t portFrequency[8] = {};
}

 
ISR(TIMER1_COMPA_vect) {
//    PORTD = nextPulseState[bufferPointer];
//    auto x = nextPulseState[bufferPointer];
//    bufferPointer = bufferPointer + 1;
}
 
void setBaseFrequencyDivident(uint16_t baseDivident) {
    OCR1AH = (uint8_t) (baseDivident >> 8);
    OCR1AL = (uint8_t) baseDivident;
}
 
int main(void)
{
    GPIOR0 = 0x00;
    auto z = GPIOR1;
    
    // PORTD fuer Pulse-Kanaele
    DDRD = 0xFF;
    
    // Timer1 mit vollem CPU Takt und OCIA im CTC Modus
    TCCR1B = (1 << WGM12) | (1 << CS10);
    TIMSK1 = (1 << OCIE1A);
    sei();
    
    // Setze default Frequenzbereich
    setBaseFrequencyDivident(160); // 16MHz / 160 -> 100.000 Hz
    
    // Debuging
    portFrequency[0] = 2;
    portFrequency[1] = 2;
    portFrequency[2] = 2;
    portFrequency[3] = 2;
    portFrequency[4] = 2;
    portFrequency[5] = 2;
    portFrequency[6] = 2;
    portFrequency[7] = 2;
    
    
    while (true) {
        // Berechne werte fuer PROTD and the OCR
        for (uint8_t i = 0; i < 256; i++) {
            uint16_t bufferPosition = ((uint16_t) bufferPointer + i);
            uint8_t nextState = 0;
            nextState |= ((bufferPosition % portFrequency[0] == 0) << 0);
            nextState |= ((bufferPosition % portFrequency[1] == 0) << 1);
            nextState |= ((bufferPosition % portFrequency[2] == 0) << 2);
            nextState |= ((bufferPosition % portFrequency[3] == 0) << 3);
            nextState |= ((bufferPosition % portFrequency[4] == 0) << 4);
            nextState |= ((bufferPosition % portFrequency[5] == 0) << 5);
            nextState |= ((bufferPosition % portFrequency[6] == 0) << 6);
            nextState |= ((bufferPosition % portFrequency[7] == 0) << 7);
            nextPulseState[bufferPosition & 0xff] = nextState;
        }
    }
}
