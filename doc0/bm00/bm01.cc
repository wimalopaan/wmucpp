#include <mcu/avr.h>
#include <mcu/pgm/pgmstring.h>
#include <algorithm>

const char s1[] PROGMEM = "abcdefghi";
const char s2[] PROGMEM = "abcdefghi";

char x[100];
etl::Char y[100];

const auto p1 = "abcdefghi"_pgm;
const auto p2 = "abcdefghi"_pgm;

int main() {
    strcpy_P(x, s1);
    strcpy_P(x, s2);
    
//    std::copy(std::begin(p1), std::end(p1), y);
//    std::copy(std::begin(p2), std::end(p2), y);
}
