#include <avr/io.h>
#include <avr/interrupt.h>

//#define XTAL        8e6         // 8MHz

#define F_TIMER1        100          // Timer 1 frequency /Hz

#define ENCODER_PIN PINC
#define PHASE_A     (1<<PC2)
#define PHASE_B     (1<<PC4)

//#define LEDS_DDR     DDRF
//#define LEDS        PORTF           // LEDs against VCC

volatile int8_t enc_delta;          // -128 ... 127
volatile int8_t last;

struct Counter
{
    int32_t neu {0};
    int32_t alt {0};
} count;


void encode_init() { 
    int8_t new_value, tmp; // new
    tmp = ENCODER_PIN;
    new_value = 0;  
    if ( tmp & PHASE_A ) new_value = 3;
    if ( tmp & PHASE_B ) new_value ^= 1;      // convert gray to binary
    last = new_value;                         // power on state
    enc_delta = 0;
    
    // put your setup code here, to run once:
    // Timer 1, CTC mode 4, prescaler  64
    TCCR1A  = 0;
    TCCR1B  = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A   = (F_CPU / (256L * F_TIMER1)) - 1;
    TIMSK1 |= (1 << OCIE1A);
}

ISR( TIMER1_COMPA_vect ) {           // 1ms for manual movement
    int8_t neu, diff, tmp;
    
    tmp = ENCODER_PIN;
    neu = 0;
    if ( tmp & PHASE_A ) neu  = 3;
    if ( tmp & PHASE_B ) neu ^= 1;   // convert gray to binary
    diff = last - neu;               // difference last - new
    if ( diff & 1 ) {                // bit 0 = value (1)
        last = neu;                    // store new as next last
        enc_delta += (diff & 2) - 1;   // bit 1 = direction (+/-)
    }
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

////////////////////////////////////////////////////////////////////////////////
// void setup()
//
void setup()
{
    //  Serial.begin( 9600 );
    
    PORTC |= PHASE_A | PHASE_B;       // activate internal pull up resistors
    //  LEDS_DDR = 0xFF;
    encode_init();
    sei();
} // Ende void setup()


void loop()
{
    count.neu += encode_read(4);          // read a single step encoder
    
    if (count.alt != count.neu)
    {
        //    Serial.print(F("count = "));
        //    Serial.println(count.neu);
        //    LEDS = count.neu;
        count.alt = count.neu;
    }
    
} // Ende void loop()

int main() {
    
}
