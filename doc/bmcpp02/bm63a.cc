#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>


volatile std::array<char, 6> a{'a', 'b', 'c', '\0'};
//volatile std::array<char, 6> b;

int main() {
    std::array<char, 6> b;
    std::copy(std::begin(a), std::end(a), std::begin(b));
    return b[0] + b[1] + b[2] + b[3] + b[4] + b[5];
}
