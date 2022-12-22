#include <avr/io.h>

//#include "bm30.h"

extern "C" {
 extern "C" {
    void f();
 }
}

#include <type_traits>

struct Bits {
    uint8_t bit0:1;
    uint8_t bit1:1;
    uint8_t bit2:1;
    uint8_t bit3:1;
    uint8_t bit4:1;
    uint8_t bit5:1;
    uint8_t bit6:1;
    uint8_t bit7:1;
};

struct S1 {
    S1(int x = 0) : x{x} {}
    int x{};
};

typedef S1 S2;

void S1() {
}

template<typename T>
auto foo(T a) {
//    T::_;
    return (struct S1){a}; // Assoziativität (notwendig bei Ausdrücken)
}

int main() {
    foo((struct S1){}); // Assoziativität (notwendig bei Ausdrücken)
    
    struct S1 s1; // Disambiguation: S1 s1; funktioniert nicht mehr, wenn Funktion S1() deklariert ist
    foo(s1);
    
    struct S1 s2{2};
    foo(s2);
    
    foo(1);

//    struct S2 s20; // typedef
    S2 s21;    
}
