#include <cstdint>

volatile uint8_t x;

template<typename T>
[[gnu::noinline]] void f(const T& v);

template<typename T>
void f(const T& v) {
    x = v / 2;
}
uint8_t a = 10;
uint16_t b = 20;

class Test{
  public:
    enum class Testenum;
};

enum class Test::Testenum {A, B};

int main() {
    f(a);
    f(b);
}
