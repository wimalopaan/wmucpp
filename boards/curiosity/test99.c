#include <avr/io.h>

int main() {
    PORTF.DIRSET = 0xff;
    PORTF.OUTCLR = 0xff;
    
    while(1) {
        PORTF.OUTTGL = 0xff;
        
    }
    
}
