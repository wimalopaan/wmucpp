#include "util/meta.h"

typedef unsigned char DataType;
volatile DataType x;
volatile DataType y;

int main() {
    DataType v = 0;
    y = [&]<auto... II>(std::index_sequence<II...>){
            ((v <<= 1, v |= (x & 0x01), (void)II), ...);
            return v;
    }(std::make_index_sequence<2>{});
    
}
