#include <avr/io.h>
#include <avr/interrupt.h>

int main(void)
{
    DDRA = 0xff;
//    PORTA = 0xff;
    
  // Timer 0 konfigurieren
  TCCR2B = (1<<CS01); // Prescaler 8

  // Overflow Interrupt erlauben
  TIMSK2 |= (1<<TOIE2);

  // Global Interrupts aktivieren
  sei();

  while(1)
  {
    /* Sonstige Aktionen */
  }
}

ISR (TIMER2_OVF_vect)
{
    PORTA ^= 0xff;
}
