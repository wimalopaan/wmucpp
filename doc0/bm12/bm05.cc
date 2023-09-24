#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define USE_INTERRUPTS

#define F_TIMER1        100          // Timer 1 frequency /Hz

#define ENCODER_PIN PORTA_IN
#define PHASE_A     (1<<5)
#define PHASE_B     (1<<6)

volatile int8_t enc_delta;          // -128 ... 127
static int8_t last;

const int8_t table[16] PROGMEM = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};    
 
void encode_init(void) {
    // put your setup code here, to run once:
    // Timer 1, CTC mode 4, prescaler  64
//    TCCR1A  = 0;
//    TCCR1B  = (1 << WGM12) | (1 << CS11) | (1 << CS10);
//    OCR1A   = (F_CPU / (256L * F_TIMER1)) - 1;
//    TIMSK1 |= (1 << OCIE1A);
}

void encoder_periodic() {
    static int8_t last=0;           // alten Wert speichern
     uint8_t tmp;
 
     tmp = ENCODER_PIN;
     last = (last << 2)  & 0x0F;
     if (tmp & PHASE_A) last |= 2;
     if (tmp & PHASE_B) last |= 1;
     enc_delta += pgm_read_byte(&table[last]);
}

// read 1, 2, or 4 step encoders
int8_t encode_read( uint8_t klicks )  {
    int8_t val;
    
    // atomic access to enc_delta
    const uint8_t saveSREG {SREG};
    cli();
    val = enc_delta;
    switch (klicks) {
    case  2: enc_delta = val & 1; val >>= 1; break;
    case  4: enc_delta = val & 3; val >>= 2; break;
    default: enc_delta = 0; break;
    }
    SREG = saveSREG;
    return val;                   // counts since last call
}


int main() {
    encode_init();
    while(true) {
        int32_t count = 0;
#ifndef USE_INTERRUPTS
        encoder_periodic();
#endif
        count += encode_read(4);          // read a single step encoder
    }
}

#ifdef USE_INTERRUPTS
ISR( TIMER1_COMPA_vect ) {           // 1ms for manual movement
    encoder_periodic();
}
#endif
