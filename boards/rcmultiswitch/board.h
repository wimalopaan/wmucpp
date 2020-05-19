#pragma once

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/event.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>

#include <external/solutions/series01/sppm_in.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/ibus/ibus.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using led7 = Pin<Port<B>, 3>; 
using led6 = Pin<Port<A>, 7>; 
using led5 = Pin<Port<A>, 5>; // tca0 wo5
using led4 = Pin<Port<A>, 4>; // tca0 wo4
using led3 = Pin<Port<A>, 3>; // tca0 wo3
using led2 = Pin<Port<B>, 2>; // tca0 wo2
using led1 = Pin<Port<B>, 1>; // tca0 wo1
using led0 = Pin<Port<B>, 0>; // tca0 wo0

using ppmIn = Pin<Port<A>, 2>; 

using ledList = Meta::List<led0, led1, led2, led3, led4, led5, led6, led7>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using tcaPosition    = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using tcb0Position    = Portmux::Position<Component::Tcb<0>, Portmux::Default>;

using pwm = PWM::DynamicPwm<tcaPosition>;

using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position>>;

namespace  {
    constexpr auto dt = 2_ms;
    constexpr auto fRtc = 128_Hz;
    
    constexpr double Ro = 4'700;
    constexpr auto RPs = []{
        const std::array<double, 3> Rx {4'700, 10'000, 20'000};
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

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using ppm_channel = Event::Channel<0, Event::Generators::Pin<ppmIn>>; 
using ppm_user = Event::Route<ppm_channel, Event::Users::Tcb<0>>;
using evrouter = Event::Router<Event::Channels<ppm_channel>, Event::Routes<ppm_user>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<6, 0x1e>>; // 1e = temp

namespace  {
    uint8_t getConfigValue() {
        constexpr auto ch0 = adcController::index_type{0};
        const auto cv = adcController::value(ch0).toInt();
        for(uint8_t i = 0; i < JumperIntervalls.size(); ++i) {
            if ((cv >= JumperIntervalls[i].first) && (cv <= JumperIntervalls[i].second)) {
                return i;
            }
        }        
        return std::numeric_limits<uint8_t>::max();
    }
}
