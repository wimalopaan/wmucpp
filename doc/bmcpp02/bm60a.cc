#include <cstdint>
#include <cstddef>
#include <array>

template<uint8_t... II, typename A, typename B>
inline static bool compare(const A& a, const B& b) {
    return ((a[II] == b[II]) && ...);
}

volatile std::array<char, 4> a{'a', 'b', 'c', '\0'};
//volatile std::array<char, 4> b{'a', 'b', 'c', '\0'};

int main() {
//    std::array<char, 4> b{'a', 'b', 'c', '\0'};
    const char* b = "abca";
    return compare<0, 1, 2>(a, b);
}
