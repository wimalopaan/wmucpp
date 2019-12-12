#include <tuple>
#include <cassert>
#include <etl/types.h>
#include <etl/tuple.h>

template<typename P, typename L>
struct Note {};

using a1 = Note<std::integral_constant<uint16_t, 100>, std::integral_constant<uint16_t, 200>>;
using a2 = Note<std::integral_constant<uint16_t, 200>, std::integral_constant<uint16_t, 300>>;
using a3 = Note<std::integral_constant<uint16_t, 300>, std::integral_constant<uint16_t, 400>>;
using a4 = Note<std::integral_constant<uint16_t, 400>, std::integral_constant<uint16_t, 500>>;
using a5 = Note<std::integral_constant<uint16_t, 500>, std::integral_constant<uint16_t, 600>>;

using notes = std::tuple<a1, a2, a3, a4, a5, a5, a4, a3, a2, a1, a1, a2, a3, a4, a5, a5, a4, a3, a2, a1>;

volatile uint16_t r1;
volatile uint16_t r2;

int main() {
    etl::uint_ranged_circular<uint8_t, 0, std::tuple_size_v<notes> - 1> index;
    while(true) {
        etl::visitAt(notes{}, index.toInt(), []<typename P, typename L>(Note<P, L>){
                          r1 = P::value;
                          r2 = L::value;
                          return true;
        });
        ++index;
    }
    
}
