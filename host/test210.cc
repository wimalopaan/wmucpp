#include <algorithm>
#include <array>

void fill1(char8_t* const p, size_t const n) {
    std::fill(p, p + n, 0);
}

void fill2(char* const p, size_t const n) {
    std::fill(p, p + n, '\0');
}

template<typename T, size_t N>
[[gnu::noinline]]
void fill3(std::array<T, N>& a) {
    std::fill(std::begin(a), std::end(a), T{});
}

int main() {
    std::array<char, 1000> a;

    fill3(a);


   std::array<char8_t, 1000> a2;

    fill3(a2);
}
