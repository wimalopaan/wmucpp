#include <stdint.h>

uint8_t i = 0;
uint8_t off = 5;

void inc(){
  i += off;
}


int main(){
  while (true) {
    inc();
  }
}
