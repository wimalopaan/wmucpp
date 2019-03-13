#include <avr/io.h>

#include <cstdint>
#include <limits>

struct F {
    inline static auto& port = PORTF.OUT;
    
    static inline auto& get(){
        return *((uint8_t*)&port + 4);
    }
    
};

int main() {
    F::get() = 0xff;
}
