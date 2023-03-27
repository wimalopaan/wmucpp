#include <avr/io.h>
#include <avr/pgmspace.h>

#include <stdbool.h>

#define puts_rom(str)      (uart_puts_rom(PSTR(str)))

static void uart_init(void) {
    // stimmt nur bei portmux-default
    PORTB.DIR |= 1 << 2;                
    PORTB.DIR &= (register8_t) ~(1 << 3);
    PORTB.PIN3CTRL |= 0x08;          
    
    USART0.BAUD = 4166; // Blödsinn
    
    USART0.CTRLB |= USART_TXEN_bm;         
    USART0.CTRLB |= USART_RXEN_bm; 
} 
static void uart_putchar(const char c) {
    while (!(USART0.STATUS & USART_DREIF_bm));
    USART0.TXDATAL = c;
}

static void uart_puts_rom(const char* dataPtr) {
    for (uint8_t c = '\0'; (c = pgm_read_byte(dataPtr)); ++dataPtr)
        uart_putchar(c);
}
static void puts_ram(const char* c) {
    while (*c) {
        uart_putchar(*c++);
    }
}

const char str1[]= "\n\r Hello World\n\n\r";

int main(void) {
    const char str2[]= "\n\r Hallo uC-Community\n\r";
    CCP= 0xd8; CLKCTRL.MCLKCTRLB= 0;    // Clk-Divider = 1 => F_CPU = 20 MHz
    uart_init();
    puts_rom("\n\r Hallo Welt");         // funktioniert
    puts_ram(str1);                      // funktioniert
    puts_ram(str2);                      // funktioniert nicht !
    puts_ram("\n\r Bonjour le monde");   // funktioniert nicht !
    while(true);
}

