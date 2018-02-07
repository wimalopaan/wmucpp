#include <cstdint>
#include <cstddef>
#include <array>

template<uint8_t... II, typename A, typename B>
inline static void  copy(A& a, const B& b) {
    ((a[II] = b[II]),...);
}

volatile std::array<char, 6> a{'a', 'b', 'c', '\0'};
//volatile std::array<char, 6> b{'a', 'b', 'c', '\0'};

int main() {
    std::array<char, 6> b{'a', 'b', 'c', '\0'};
    copy<0, 1, 2, 3, 4, 5>(b, a);
    return b[0] + b[1] + b[2] + b[3] + b[4] + b[5];
}
