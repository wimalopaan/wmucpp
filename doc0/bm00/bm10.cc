#include <type_traits>

volatile uint8_t r;

// das folgende wäre im echten Leben eine Pin-Klasse, die eh schon da ist.
template<bool B> using ActiveHigh = std::integral_constant<bool, B>;

template<typename ActiveHigh> 
struct In {
    inline static void activate(bool b) {
        if (b ^ ActiveHigh::value) {
            // whatever todo for high()            
            r = 1;
        }    
        else {
            // whatever todo for low()
            r = 0;
        }
    }
};

// der tatsächlich benötigte Code

using In1 = In<ActiveHigh<true>>;
using In2 = In<ActiveHigh<false>>;

namespace {
    volatile bool analogRichtung{};
    volatile bool analogPwm{};
}

int main() {
    if (analogRichtung) {
        In1::activate(analogPwm);
    }
    else {
        In2::activate(analogPwm);
    }
}
