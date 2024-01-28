#define NDEBUG

#include <cmath>

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/event.h>
#include <mcu/internals/adc.h>

#include <external/hal/adccontroller.h>
#include <external/solutions/tick.h>
#include <external/solutions/series01/sppm_in.h>

#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

namespace  {
using namespace External::Units::literals;
constexpr auto fRtc = 1000_Hz;
}

#include "stepper.h"
#include "devices.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using devices = Devices<Board412_FOC_01>;

template<typename Devs>
struct GFsm {
    using devs = Devs;
    using pwm = devs::pwm;

    inline static constexpr uint16_t period = 1024; // 16KHz
    inline static constexpr uint16_t maxPeriodValue = period - 1;

    inline static constexpr uint16_t steps = 512;

    template<size_t Steps>
    struct Generator {
        constexpr auto operator()() {
            std::array<uint16_t, Steps> data;
            for(uint16_t i = 0; i < Steps; ++i) {
                data[i] = maxPeriodValue * (1.0 + sin((i * 2 * M_PI) / Steps)) / 2.0;
            }
            return data;
        }
    };
    using Sine10 = AVR::Pgm::Util::Converter<Generator<steps>>::pgm_type;

    using index_type = etl::uint_ranged_circular<uint16_t, 0, steps - 1> ;

    static inline void init() {
        pwm::template init<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
        pwm::period(period);
        pwm::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();

        // pwm::template duty<Meta::List<AVR::PWM::WO<0>>>(500);
        // pwm::template duty<Meta::List<AVR::PWM::WO<1>>>(500);
        // pwm::template duty<Meta::List<AVR::PWM::WO<2>>>(500);

        // devs::pa7::template dir<AVR::Output>();


    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
        pwm::template duty<Meta::List<AVR::PWM::WO<0>>>(Sine10::value(index0));
        pwm::template duty<Meta::List<AVR::PWM::WO<1>>>(Sine10::value(index1));
        pwm::template duty<Meta::List<AVR::PWM::WO<2>>>(Sine10::value(index2));

        ++index0;
        ++index1;
        ++index2;
    }

    inline static index_type index0{0};
    inline static index_type index1{steps / 3};
    inline static index_type index2{(2 * steps) / 3};
};

using gfsm = GFsm<devices>;

int main() {
    devices::portmux::init();
    devices::ccp::unlock([]{
#if (F_OSC == 20000000)
        devices::clock::prescale<1>();
#elif (F_OSC == 10000000)
        devices::clock::prescale<2>();
#else
#error "wrong F_OSC"
#endif
    });
    devices::systemTimer::init();

    gfsm::init();

    while(true) {
        gfsm::periodic();
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

