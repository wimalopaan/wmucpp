#include <avr/io.h>
#include "main.h"
#include "UART.h"
// Command: 
//    bit 7 = command flag
//    bit 6 =  ESC command
//    bit 5 =  maintenance mode
//    bit 4:2 = motor index (up to 8 motors)
//    bit 1:0 = setpoint bits 8 and 7 (up to 512 positions)

void openSerial()
{
    /* initialize UART*/
  UBRRH = (((F_CPU/BAUD)/16)-1)>>8; 	// set baud rate
  UBRRL = (((F_CPU/BAUD)/16)-1);
  UCSRB = (1<<RXEN)|(1<<TXEN);  // enable Rx & Tx
  UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // config USART; 8N1
}

int getch(void)
{
  if(!(UCSRA & _BV(RXC))) 
  {
    return -1;
  }
  else
  { 
    return (UDR);
  }
}


void putch(char ch)
{
  while (!((UCSRA) & _BV(UDRE)));
  UDR = ch;
}

void putln(char ch)
{
  while (!((UCSRA) & _BV(UDRE)));
  UDR = ch;
  while (!((UCSRA) & _BV(UDRE)));
  UDR = '\n';
}


