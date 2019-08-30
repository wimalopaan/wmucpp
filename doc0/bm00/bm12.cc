#include <stdint.h>

int foo(int x) {
  constexpr int shift = 6 - 13;
  if constexpr(shift < 0) {
    return x >> -shift;
  } else {
    return x << shift;
  }
}

int main() {
}
