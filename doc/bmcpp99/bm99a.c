/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>

int main(void)
{

  DDRB = 0xff;                        
  DDRC = 0xff; 
  PORTD = 0xff;                      
//  GICR |= (1 << INT1);               
  MCUCR |= (1 << ISC10)|(1<<ISC11);              
  sei();  
                            
  while(1){  
      PORTC=0xff;
      _delay_ms(1);
      PORTC=0x00;
      _delay_ms(1);
  }                                                    
}

ISR(INT1_vect, ISR_NAKED) {
  PORTB=0xff;
  PORTB=0x00;
}
