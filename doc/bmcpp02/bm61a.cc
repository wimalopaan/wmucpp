#include <cstdint>
#include <cstddef>
#include <array>

#include <string.h>

volatile std::array<char, 4> a{'a', 'b', 'c', '\0'};
//volatile std::array<char, 4> b{'a', 'b', 'c', '\0'};

int main() {
//    std::array<char, 4> b{'a', 'b', 'c', '\0'};
    const char* b = "abca";
    return strncmp((const char*)&a[0], (const char*)&b[0], a.size);
//    return strcmp((const char*)&a[0], (const char*)&b[0]);
}
