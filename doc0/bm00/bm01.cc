#include <etl/type_traits.h>
#include <etl/fixedpoint.h>

#include <mcu/avr.h>

using FP = etl::FixedPoint<int16_t, 12>;

volatile FP x{1.0};
volatile FP z;

volatile int16_t a;
volatile int16_t b;

int main() {
    using namespace etl;
    constexpr etl::FixedPoint<int16_t, 12> c{FP::one/3200.0};
    
    z = FP::fromRaw(3200);
    z = z * c;

//    int32_t d = (int32_t)a * 0x5e3a;
    
//    b = d >> 12;
    
//    auto y = x * c;

//    y += FP{0.5};
}
