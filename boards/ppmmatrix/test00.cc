//#define NDEBUG

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

int main() {
    for(const auto& c : "def"_pgm) {
        ppmInPin::toggle();
    }
}

