#include <stdint.h>
#include <stddef.h>

volatile uint8_t r;

typedef int a_t;
#define A_SIZE 3

//uint8_t rr;
//static const a_t a[A_SIZE] = {
//  [0] = rr,
//  [1] = rr,
//  [2] = rr
//};

//a_t f(const uint8_t i) {
//    return a[i];
//}

uint8_t i = 100;  

typedef struct {
    const unsigned Anzahl;
    uint8_t Werte[];
} Muster_t;
 
//static Muster_t MusterA = { 3, { 1 } };
static Muster_t MusterB = { 1000, {
                         [0] = 1,
                         [1] = 2,
                         [2] = 3,
                         [42] = 42
                     } 
                   };

int main() { 
    int a[i];
//    return MusterB.Werte[42];
//    return sizeof(MusterB.Werte);
//    return f(i);
}
