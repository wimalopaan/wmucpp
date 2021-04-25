#include <cstdint>

constexpr int x{};

const int y{};

int main() {
    *const_cast<int*>(&x) = 42;
}
