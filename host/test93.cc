#include <cstdint>
#include <cstddef>
#include <array>

constexpr int fkt(size_t value) {
    return value;
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
}
