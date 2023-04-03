/*  
    7-SEGMENT-ANZEIGE  Multiplexbetrieb  mit drei 7-Segment-Anzeigen, CA,
    Ansteuerung an PORTB0-B2 mit PNP Transistoren 
    ATMega48
 */

//#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

uint16_t zahl;
uint8_t  anzeige[3];
const uint8_t segmenttable[11] = 
    { 0x03, 0xF3, 0x25, 0x0D, 0x99, 0x49, 0x41, 0x1F, 0x01, 0x19, 0xFE };

#define T1      0xFE  // Basis Transistor 1 (PNP) Einerstelle
#define T2      0xFD  // Basis Transistor 2  Zehnerstelle
#define T3      0xFB  // Basis Transistor 3 Hunderterstelle
#define T_AUS   0xFF  // alle aus

void wert_zerlegen (int display_value)
{
    anzeige[2] = display_value % 10; display_value /= 10;
    anzeige[1] = display_value % 10; display_value /= 10;
    anzeige[0] = display_value % 10;
}

int main (void)
{
  
    DDRD  = 0xFF; // PORTD auf Ausgang setzen
    DDRB  = 0xFF;
    PORTD = 0xFF; // Ausgabeport für 7-Segment-Ziffern
    PORTB = 0xFF; // Ansteuerung von T1 bis T3  
    zahl = 345;
    
    while(1)
    {
    
        wert_zerlegen(zahl);
    
        PORTB = T_AUS;
        PORTD = segmenttable[anzeige[2]];
        PORTB = T1;
        _delay_ms(7);
    
        
        PORTB = T_AUS;
        PORTD = segmenttable[anzeige[1]];
        PORTB = T2;
        _delay_ms(7);
    
        PORTB = T_AUS;
        PORTD = segmenttable[anzeige[0]];
        PORTB = T3;
        _delay_ms(7);
    }
    return 0;
}
