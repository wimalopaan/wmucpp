//#define NDEBUG

#ifndef NDEBUG
static unsigned int assertKey{1234};
#endif
  
#include <mcu/avr.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

#include "devices.h"
#include "busdevs.h"
#include "esc.h"
#include "fbservo.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;
 
#ifndef NDEBUG
namespace xassert {
    etl::StringBuffer<160> ab;
    etl::StringBuffer<10> aline;
    bool on{false};
}
#endif

template<typename PPM, uint8_t N>
struct PPMAdapter {
    using ranged_type = PPM::ranged_type;
    using index_t = PPM::index_type;
    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(index_t{N}, v.toInt());
    }
};

template<typename ADC, uint8_t N>
struct ADCAdapter {
    using value_type = ADC::value_type; 
    using index_type = ADC::index_type; 
    inline static value_type value() {
        return ADC::value(index_type{N});
    }
};

template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    
    using bus_type = BusDevs::bus_type;
    using devs = BusDevs::devs;
    using Timer = devs::systemTimer;
    
    using lut2 = devs::lut2;
    using Adc = devs::adcController;
    
    using servo = BusDevs::servo;
    using servo_pa = BusDevs::servo_pa;
    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    using value_t = servo_pa::value_type;
    static inline constexpr value_t chThreshHPos = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshLPos = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};    
    static inline constexpr value_t chThreshHNeg = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshLNeg = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 6};    
    
//    std::integral_constant<uint16_t, chThreshHPos.toInt()>::_;
    
//    using Sensor = BusDevs::sensor;
//    using sensor_pa = Sensor::ProtocollAdapter;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    using adc_i_t = Adc::index_type;
  
    // pwm0: servo0
    // pwm1: servo1
    // pwm2: esc0
    // pwm3: esc1
    
    using pwm0_2 = devs::pwm0_2;
    using pwm3 = devs::pwm3;
    using pwm4 = devs::pwm4;
    using pwm5 = devs::pwm5;
//    using pwm_t  = pwm0_2::ranged_type;
    
    using pwm0 = PPMAdapter<pwm0_2, 0>;
    using adc0 = ADCAdapter<Adc, 0>;
    using servo_0 = ::Servo<pwm0, adc0, Timer>;
    using esc0 = EscFsm<pwm0_2, 2, Timer, value_t, servo_0>;
    
    using pwm1 = PPMAdapter<pwm0_2, 1>;
    using adc1 = ADCAdapter<Adc, 1>;
    using servo_1 = ::Servo<pwm1, adc1, Timer>;
    using esc1 = EscFsm<pwm3, 0, Timer, value_t, servo_0>;
    
    static inline void init(const bool) {
        servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
        servo::rxInvert(true);
        pwm0_2::init();
        pwm3::init();
        pwm4::init();
        pwm5::init();
        
    }
    static inline void periodic() {
        servo::periodic();
    }
    
    enum class State : uint8_t {Undefined, Init};
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> debugTicks{500_ms};
    
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        
        const auto oldState = mState;
        ++mStateTick;
        mStateTick.on(debugTicks, []{
            etl::outl<terminal>("ch0: "_pgm, servo_pa::value(0).toInt());
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                break;
            }
        }
        
    }

private:
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick{};
    
};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        using terminal = devs::terminal;
        using systemTimer = devs::systemTimer;
        using gfsm = GlobalFsm<devs>;
        
        gfsm::init(inverted);
        
        etl::outl<terminal>("rc720_00"_pgm);
        
        while(true) {
            gfsm::periodic(); 
            systemTimer::periodic([&]{
                gfsm::ratePeriodic();
            });
        }
    }
};

using devices = Devices<>;
using app = Application<External::Bus::NoBus<devices>>;

int main() {
    devices::init();
    app::run();
//    scanner::run();
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    xassert::ab.clear();
    xassert::ab.insertAt(0, expr);
    etl::itoa(line, xassert::aline);
    xassert::ab.insertAt(20, xassert::aline);
    xassert::ab.insertAt(30, file);
    xassert::on = true;
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
