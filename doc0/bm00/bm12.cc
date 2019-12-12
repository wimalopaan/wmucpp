#include <limits>
#include <array>

std::array<uint8_t, 10> a {1, 2, 3};

volatile uint8_t r;

int main() {
    for(decltype(a)::size_type i{0}; i < a.size(); ++i) {
        r;
    }
    
    for(uint16_t i{0}; i < a.size(); ++i) {
        r;
    }
}
