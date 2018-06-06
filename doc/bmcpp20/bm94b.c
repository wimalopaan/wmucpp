#define __PROG_TYPES_COMPAT__

#include <stdint.h>
#include <avr/pgmspace.h>

#define RVCC    10.0    // 10k to VCC
#define RROW    1.0     // 1k between rows
#define RCOL    3.9     // 3k9 between colums
#define NROW    4       // number of rows
#define ADCRES  256     // ADC resolution
#define NKEY    12

#define ADCVAL(n)       ADCRES*(1-RVCC/((n)%NROW*RROW+(n)/NROW*RCOL+RVCC))

#define THRESHOLD(n)    (uint16_t)((ADCVAL(n)+ADCVAL(n+1))/2)

const prog_uint8_t THRESHOLDS[] = {   THRESHOLD(11),
                                THRESHOLD(10),
                                THRESHOLD(9),
                                THRESHOLD(8),
                                THRESHOLD(7),
                                THRESHOLD(6),
                                THRESHOLD(5),
                                THRESHOLD(4),
                                THRESHOLD(3),
                                THRESHOLD(2),
                                THRESHOLD(1),
                                THRESHOLD(0),
                                0 };          // end of table

#if 0
uint16_t key_no( uint8_t adcval ) {
  uint16_t num = 1<<NKEY;
  const prog_uint8_t * thr = THRESHOLDS;

  while( adcval < pgm_read_byte( thr )){
    thr++;
    num >>= 1;
  }
  return num & ~(1<<NKEY);
}
#else
uint8_t key_no( uint8_t adcval ) {
  uint8_t num = 0;
  const prog_uint8_t * thr = THRESHOLDS;

  while( adcval < pgm_read_byte( thr )){
    thr++;
    num++;
  }
  return num ;
}
#endif

volatile uint8_t value = 100;
volatile uint8_t result = -1;

int main() {
    result = key_no(value);
}
