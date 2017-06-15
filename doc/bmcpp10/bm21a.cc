#include <stdint.h>
#include "container/pgmstring.h"

volatile uint8_t x;

volatile uint8_t y1;
volatile uint8_t y2;

void f1(PgmStringView s) {
    for(uint8_t i = 0; s[i] != '\0'; ++i) {
        x += s[i] + y1;    
    }
}
void f2(PgmStringView s) {
    for(uint8_t i = 0; s[i] != '\0'; ++i) {
        x += s[i] + y2;    
    }
}

int main() {
    f1("ok"_pgm);    
    f2("nok"_pgm);    
    f1("nok"_pgm);    
    f2("ok"_pgm);    
}
