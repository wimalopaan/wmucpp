#define NDEBUG

#define LEARN_DOWN

#ifndef GITMAJOR
# define VERSION_NUMBER 0001
#endif

#ifndef NDEBUG
static unsigned int assertKey{1234};
#endif

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
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>

#include <mcu/pgm/pgmarray.h>

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
#include <stdlib.h>

#include <etl/output.h>
#include <etl/meta.h>
#include <etl/control.h>

#include "driver.h"
#include "encoder.h"
#include "devices.h"
#include "busses.h"


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

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

template<typename T>
constexpr auto cyclic_diff(const T& a, const T& b) {
    using diff_t = std::make_signed_t<typename T::value_type>;
    
    constexpr diff_t adelta2 = (T::Upper - T::Lower) / 2;
    constexpr diff_t adelta  = (T::Upper - T::Lower);
//    std::integral_constant<diff_t, adelta2>::_;
    
    const diff_t as(a);
    const diff_t bs(b);
    
    const diff_t diff = as - bs;
    if (diff > 0) {
        if (diff > adelta2) {
            return as - (bs + adelta);
        }
        else {
            return as - bs;
        }
    }
    else {
        if (diff < -adelta2) {
            return as - (bs - adelta);
        }
        else {
            return as - bs;
        }
    }
}

template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    
//    using bus_type = BusDevs::bus_type;
    using devs = BusDevs::devs;
    using Timer = devs::systemTimer;
    
    using blinkLed = External::Blinker2<typename devs::led, Timer, 100_ms, 2500_ms>;
    using count_type = blinkLed::count_type;
    
    using enable = devs::enable;
    
    using dbg = devs::dbgPin;
    
    using driver = Driver<typename devs::pwm, std::integral_constant<size_t, 2048>, std::integral_constant<uint8_t, 11>>;
    
    using encoder = devs::encoder;
    using angle_t = encoder::angle_type;
    
    using lut3 = devs::lut3;
    using Adc = devs::adcController;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    using adc_i_t = Adc::index_type;
    
    enum class State : uint8_t {Undefined, Init, Enable, Rotate, StartRotate, EndRotate};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> enableTicks{1000_ms};
    static constexpr External::Tick<Timer> debugTicks{300_ms};
    
    static inline void init() {
        TermDev::template init<AVR::BaudRate<115200>>();
        
        blinkLed::init();
        enable::init();
        
        Adc::template init<false>(); // no pullups
        Adc::mcu_adc_type::nsamples(4);
        
        encoder::init();
        driver::init();
        
        dbg::template dir<Output>();
    }
    
    static inline void periodic() {
        TermDev::periodic();
        Adc::periodic();
        
        encoder::periodic();
        driver::periodic();
        
        Adc::periodic();
        
        encoder::template read<true>(); // increased time
    }
    
    static inline void ratePeriodic() {
        dbg::toggle();
        
        driver::ratePeriodic();
        
        blinkLed::ratePeriodic();
        
        const auto oldState = mState;
        ++mStateTick;
        
        const angle_t angle = encoder::angle();
        
        (++mDebugTick).on(debugTicks, [&]{
            etl::outl<terminal>("ma: "_pgm, angle.toInt(), " eo: "_pgm, eoffset.toInt(), " di: "_pgm, driver::mIndex);
//            etl::outl<terminal>(mPid);
        });
        
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(enableTicks, []{
                mState = State::Enable;
            });
            break;
        case State::Enable:
            mStateTick.on(enableTicks, []{
                mState = State::StartRotate;
            });
            break;
        case State::StartRotate:
            mStateTick.on(enableTicks, []{
                mState = State::Rotate;
            });
            break;
        case State::Rotate:
            ++eangle;
            driver::angle(eangle);
            if (eangle.isTop()) {
                mState = State::EndRotate;
            }
            break;
        case State::EndRotate:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S: Init"_pgm);
                blinkLed::blink(count_type{1});
                enable::activate();
                break;
            case State::Enable:
                etl::outl<terminal>("S: Enable ea=0"_pgm);
                eangle.setToBottom();
                mangle = 0;
                driver::angle(eangle);
                break;
            case State::StartRotate:
                etl::outl<terminal>("S: Start R"_pgm);
                mangle = encoder::angle();
                break;
            case State::Rotate:
                etl::outl<terminal>("S: Rot"_pgm);
                break;
            case State::EndRotate:
                etl::outl<terminal>("S: End R"_pgm);
                break;
            }
        }
    }
private:
    static inline Control::PID<int16_t, float> mPid{1.0, 0.0, 0.0, 1000, 31};
    
    static inline angle_t mangle{0};
    static inline typename driver::angle_type eangle{0};
    static inline typename driver::angle_type eoffset{0};
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
};


template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using busdevs = BusDevs<BusSystem>;
    
    inline static void run() {
        if constexpr(External::Bus::isNoBus<BusSystem>::value) {
            using terminal = busdevs::terminal;
            using systemTimer = busdevs::systemTimer;
            using gfsm = GlobalFsm<busdevs>;
            
            gfsm::init();
            
            etl::outl<terminal>("foc_t21_hw01"_pgm);
            
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
using app = Application<External::Bus::NoBus<devices>>;

int main() {
    devices::init();
    app::run();
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
