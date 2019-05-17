#define NDEBUG

#include "board.h"

inline static constexpr uint8_t hottScale = adcController::mcu_adc_type::VRef / 0.02;
inline static constexpr auto rawMax = adcController::value_type::Upper;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;

    auto x = adcController::value(0);
    
    using conv = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t>;
    
    return conv::convert(x).value;



//    return etl::Rational::RationalDivider<uint16_t, hottScale, rawMax>::scale(x.toInt());
    
}
