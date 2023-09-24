#include <avr/io.h>
#include <avr/interrupt.h>

//#define USE_INTERRUPTS

//#define USE_DIFFERENT

#define ENCODER_PIN PORTA_IN
#define ENCODER_CFG PORTA_PINCONFIG
#define ENCODER_UPD PORTA_PINCTRLUPD
#define PHASE_A     (1<<5)
#define PHASE_B     (1<<6)
#define PHASE_C     (1<<0)
#define ENCODER2_PIN PORTC_IN
#define ENCODER2_CFG PORTC_PINCONFIG
#define ENCODER2_UPD PORTC_PINCTRLUPD

/*static */volatile int8_t enc_delta;          // -128 ... 127
/*static */int8_t last;

/*static*/ void encode_init(void) {
#ifdef USE_DIFFERENT
    ENCODER_CFG = PORT_PULLUPEN_bm;
    ENCODER_UPD = PHASE_A;    
    ENCODER2_CFG = PORT_PULLUPEN_bm;
    ENCODER2_UPD = PHASE_C ;    
#else
    ENCODER_CFG = PORT_PULLUPEN_bm;
    ENCODER_UPD = PHASE_A | PHASE_B;    
#endif
}

/*static */void encoder_periodic() {
    int8_t neu, diff, tmp;
#ifdef USE_DIFFERENT
    asm(";xxc");
    int8_t tmp1, tmp2;
    tmp1 = ENCODER_PIN;
    tmp2 = ENCODER2_PIN;
    neu = 0;
    if ( tmp1 & PHASE_A ) neu  = 3;
    if ( tmp2 & PHASE_C ) neu ^= 1;   // convert gray to binary
#else
    tmp = ENCODER_PIN;
    neu = 0;
    if ( tmp & PHASE_A ) neu  = 3;
    if ( tmp & PHASE_B ) neu ^= 1;   // convert gray to binary
#endif
    diff = last - neu;               // difference last - new
    if ( diff & 1 ) {                // bit 0 = value (1)
        last = neu;                    // store new as next last
        enc_delta += (diff & 2) - 1;   // bit 1 = direction (+/-)
    }
}

// read 1, 2, or 4 step encoders
/*static */int8_t encode_read( uint8_t klicks )  {
    int8_t val;
    
    // atomic access to enc_delta
    const uint8_t saveSREG = SREG;
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

#define LEDS_DDR    PORTD_DIR
#define LEDS        PORTD_OUT

int32_t count = 0;

int main() {
#ifdef USE_INTERRUPTS
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CCMP = 4000; // 1ms
    TCB0.CTRLA = TCB_ENABLE_bm;
    sei();
#endif
    LEDS_DDR = 0xff;

    encode_init();
    
    while(true) {
#ifndef USE_INTERRUPTS
        encoder_periodic();
#endif
        count += encode_read(2);          // read a single step encoder
        LEDS = count;
    }
}

#ifdef USE_INTERRUPTS
ISR(TCB0_INT_vect) {       
    TCB0.INTFLAGS = TCB_CAPT_bm;
    encoder_periodic();
}
#endif
