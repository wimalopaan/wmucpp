#include <tuple>
#include <etl/tuple.h> // etl::visit()
#include <mcu/pgm/pgmstring.h>

#include "bm12.h"

namespace {
    std::tuple strings{"abc"_pgm, "def"_pgm, "abc"_pgm};
}

int main() {
    
    for(auto e : Config::data()) {
        
    }
    
    uint8_t sum{};

    etl::visit(strings, [&](auto&& s){
        for(auto&& c : s) {
            sum += uint8_t(c);
        }
    });
    return sum;
}


