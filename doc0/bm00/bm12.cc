#include <avr/io.h>

#include <cstdint>
#include <limits>
#include <array>

struct F {
    inline static auto& port = PORTF.OUT;
    
    static inline auto& get(){
        return *((uint8_t*)&port + 4);
    }
    
};

enum class Index {a, b, c, d};

constexpr int lut[] = {
    [static_cast<uint8_t>(Index::a)] = 1,
    [static_cast<uint8_t>(Index::b)] = 1,
};

constexpr auto lut2 = []{
    std::array<int, 10> a;
    a[0] = 0;
    return a;
}();


int main() {
    F::get() = 0xff;
}
