#include <stdint.h>

int foo(int x) {
  const int shift = 6 - 13;
  if (shift < 0) {
    return x >> -shift;
  } else {
    return x << shift;
  }
}

int main() {
    int a={0};
}
