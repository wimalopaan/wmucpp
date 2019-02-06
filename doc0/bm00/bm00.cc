#include <etl/type_traits.h>

#include <mcu/avr.h>
#include <mcu/common/register.h>
#include <mcu/common/staticstorage.h>
#include <mcu/internals/gpiorflags.h>

using gp0 = AVR::SbiCbiRegister<DefaultMcuType::GPIOR, AVR::ComponentNumber<0>>;
using fb0 = AVR::StaticByte<AVR::ComponentNumber<0>>;
using ab0 = AVR::StaticBoolArray<AVR::ComponentNumber<0>, AVR::Size<8>>;

using fr0 = AVR::GPIORFlags<gp0, AVR::First<1>, AVR::Last<2>>;
//using fr0 = AVR::GPIORFlags<fb0, AVR::First<1>, AVR::Last<2>>;
//using fr0 = AVR::GPIORFlags<ab0, AVR::First<1>, AVR::Last<2>>;

volatile bool x;
volatile bool y;

int main() {
    while(true) {
        if (x) {
            fr0::set<0>();
        }
        if (fr0::isSet<1>()) {
            y = false;
        }
    }
}
