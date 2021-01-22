#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/syscfg.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>


namespace  {
    
    constexpr double Ro = 4'700;
    constexpr auto RPs = []{
        const std::array<double, 4> Rx {4'700, 10'000, 20'000, 40'000}; 
        std::array<double, 1 << Rx.size()> R;
        auto parallel = [](const double a, const double b){
            double z = a * b;
            double n = a + b;
            if (n != 0) {
                return z / n; 
            }
            return 0.0;
        };
        for(uint8_t i = 0; i < R.size(); ++i) {
            double r1 = 10'000'000.0;
            double r2 = 10'000'000.0;
            double r3 = 10'000'000.0;
            if (i & 0x01) {
                r1 = Rx[0];
            }
            if (i & 0x02) {
                r2 = Rx[1];
            }
            if (i & 0x04) {
                r3 = Rx[2];
            }
            const double ra = parallel(parallel(r1, r2), r3);
            R[i] = ra;
        }
        return R;
    }();
    
    constexpr auto JumperIntervalls = []{
        const double Vref = 4.3;
        const double amax = 1023;
        const double vmax = 5.0;
        std::array<std::pair<uint16_t, uint16_t>, RPs.size()> ii;
        for(uint8_t i = 0; const auto r: RPs) {
            const double v = vmax * r / (r + Ro); 
            const double vl = v * 0.96;
            const double al = vl * amax / Vref;
            const double vh = v * 1.04;
            const double ah = vh * amax / Vref;
            
            ii[i].first = std::min(amax, al);
            ii[i].second= std::min(amax, ah);
            ++i;
        }
        return ii;
    }();
       
}

int main() {
    
}
