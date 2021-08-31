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

template<typename PPM>
struct PPMAsyncAdapter {
    using ranged_type = PPM::ranged_type;
    using index_t = PPM::index_type;
    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(v.toInt());
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
    
    using sensor = BusDevs::sensor;
    using sensor_pa = sensor::ProtocollAdapter;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    using adc_i_t = Adc::index_type;
  
    enum class State : uint8_t {Undefined, Init, Run};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> debugTicks{500_ms};
    static constexpr External::Tick<Timer> pulseTicks{20_ms};

    // pwm0: servo0
    // pwm1: servo1
    // pwm2: esc0
    // pwm3: esc1
    
    using pwm0_2 = devs::pwm0_2;
    using pwm3s = devs::pwm3;
    using pwm4s = devs::pwm4;
    using pwm5s = devs::pwm5;
//    using pwm_t  = pwm0_2::ranged_type;
    
    using pwm0 = PPMAdapter<pwm0_2, 0>;
    using adc0 = ADCAdapter<Adc, 0>;
    using servo_0 = ::Servo<pwm0, adc0, Timer>;
    using pwm2 = PPMAdapter<pwm0_2, 2>;
    using esc0 = EscFsm<pwm2, Timer, value_t, servo_0>;
    
    using pwm1 = PPMAdapter<pwm0_2, 1>;
    using adc1 = ADCAdapter<Adc, 1>;
    using servo_1 = ::Servo<pwm1, adc1, Timer>;
    using pwm3 = PPMAsyncAdapter<pwm3s>;
    using esc1 = EscFsm<pwm3, Timer, value_t, servo_0>;

    using evrouter = devs::evrouter;
    
    static inline void init(const bool inverted) {
        if constexpr(External::Bus::isIBus<bus_type>::value) {
//            lut3::init(std::byte{0x00}); // low on lut3-out 
//            sensor::init();
//            sensor::uart::txOpenDrain();

            servo::template init<BaudRate<115200>>();
            etl::outl<terminal>("IB"_pgm);
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
//            lut3::init(std::byte{0x33}); // route TXD (inverted) to lut1-out 
//            sensor::init();
//            sensor::uart::txPinDisable();

//            devs::ibt::init();
//            devs::ibt::off();
            
            servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (inverted) {
                servo::rxInvert(true);
                etl::outl<terminal>("SB I"_pgm);
            }
            else {
                etl::outl<terminal>("SB"_pgm);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
        pwm0_2::template init<true>();
        
        pwm3s::init();
        pwm4s::init();
        pwm5s::init();
    }
    static inline void periodic() {
        servo::periodic();        
        servo_0::periodic();
        servo_1::periodic();
        esc0::periodic();
        esc1::periodic();
    }
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        servo_0::ratePeriodic();
        servo_1::ratePeriodic();
        esc0::ratePeriodic();
        esc1::ratePeriodic();
        
        const auto oldState = mState;
        ++mStateTick;
        ++mDebugTick;
        
        mDebugTick.on(debugTicks, []{
            etl::outl<terminal>("ch0: "_pgm, servo_pa::value(0).toInt());
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState =State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(startupTicks, []{
                mState =State::Run;
            });
            break;
        case State::Run:
            mStateTick.on(pulseTicks, []{
                pwm3s::onReload([]{
                    evrouter::template strobe<0>();
                });
                pwm4s::onReload([]{
                    evrouter::template strobe<1>();
                });
                pwm5s::onReload([]{
                    evrouter::template strobe<2>();
                });
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S I"_pgm);
                break;
            case State::Run:
                etl::outl<terminal>("S R"_pgm);
                break;
            }
        }
    }
private:
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick{};
    static inline External::Tick<Timer> mDebugTick{};
    
};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isSBus<BusSystem>::value || External::Bus::isIBus<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GlobalFsm<devs>;
            
            gfsm::init(inverted);

            etl::outl<terminal>("rc720_01"_pgm);
            
            while(true) {
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using devices = Devices<>;
using scanner = External::Scanner2<devices, Application, Meta::List<External::Bus::IBusIBus<devices>, External::Bus::SBusSPort<devices>>, AVR::FullDuplex>;
//using scanner = External::Scanner2<devices, Application, Meta::List<External::Bus::SBusSPort<devices>>, AVR::FullDuplex>;

int main() {
    scanner::run();
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
