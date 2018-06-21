#include <stddef.h>
#include <stdint.h>

uint8_t INIT1[] = "\x1b""A";
uint8_t INIT2[] = "\x1b""BB";
uint8_t INIT3[] = "\x1b""CCC";
uint8_t INIT4[] = "\x1b""DDDD";

uint8_t volatile x;

template<typename T>
void tft_wr(T* arr, size_t length) {
    x = *arr;
}

template <typename T, size_t N>
inline void tft_wr(T (&arr) [N]) {
  tft_wr(arr, N);
}

/*volatile */uint8_t a, b, c, d;

template <typename T, size_t N>
inline void init(T (&arr) [N]) {
  tft_wr(arr);

  // relativ viel zusätzlicher Code
  a = b + c; d = a + b; c = d + a; b = c + d;
  a = b + c; d = a + b; c = d + a; b = c + d;
}

int main() {
  init(INIT1);
  init(INIT2);
  init(INIT3);
  init(INIT4);
}
