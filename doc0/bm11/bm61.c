#include <stdint.h>
#include <stddef.h>

typedef int a_t;
#define A_SIZE 300

static const a_t a[A_SIZE] = {
  [ 2] = 123,
  [13] = 456,
  [75] = 789
};

a_t f(const uint8_t i) {
    return a[i];
}
 
uint8_t i = 2;

int main() {    
    return f(i);
}
