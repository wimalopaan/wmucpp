#include <cstdint>
#include <cstddef>
#include <array>
#include <iostream>

//auto l1 = [](auto v){return 2 * v;};
//decltype(l1) xx;

constexpr int fkt(size_t value) {
    return 2 * value;
}
constexpr size_t Size = 16;
constexpr auto test = []{
    std::array<int, Size> a{};
    for(size_t i = 0; i < Size; ++i) {
        a[i] = fkt(i);
    }
    return a;
}();

int main() {
    for(const auto& v : test) {
        std::cout << v << '\n';
    }
}
