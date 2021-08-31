//#define NDEBUG

#ifndef NDEBUG
//static unsigned int assertKey{1234};
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
    using timer = devs::systemTimer;
    
    using lut2 = devs::lut2;
    
    using adc = devs::adcController;
    using adc_i_t = adc::index_type;
//    adc_i_t::_;

    using config1 = devs::config1;
    using config2 = devs::config2;
    
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
    
    enum class State : uint8_t {Undefined, 
                                ShowBus,
                                CheckConfig, SearchChannel, ShowChannel, ShowChannel2,
                                Init, Calibrate1, Calibrate2, Run};
    
    static constexpr External::Tick<timer> startupTicks{100_ms};
    static constexpr External::Tick<timer> initTimeoutTicks{300_ms};
    static constexpr auto showTimeout = 1000_ms;
    static constexpr External::Tick<timer> showTimeoutTicks{showTimeout};
    
    static constexpr External::Tick<timer> debugTicks{500_ms};
    static constexpr External::Tick<timer> pulseTicks{20_ms};

    static inline constexpr value_t chThreshPosH = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshPosL = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};

    static inline constexpr value_t chThreshNegH = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshNegL = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 6};

    using blinker1 = External::Blinker2<typename devs::led1, timer, 100_ms, showTimeout>;
    using blinker2 = External::Blinker2<typename devs::led2, timer, 100_ms, showTimeout>;
    using bcount_t = blinker1::count_type;

    // pwm0: servo0
    // pwm1: servo1
    // pwm2: esc0
    // pwm3: esc1
    
    using pwm0_2 = devs::pwm0_2;
    using pwm3s = devs::pwm3;
    using pwm4s = devs::pwm4;
    using pwm5s = devs::pwm5;
//    using pwm_t  = pwm0_2::ranged_type;
    
    using pwm0 = PPMAdapter<pwm0_2, 1>;
    using adc0 = ADCAdapter<adc, 1>;
    using servo_0 = ::Servo<pwm0, adc0, timer, TermDev>;
    using pwm2 = PPMAdapter<pwm0_2, 0>;
    using esc0 = EscFsm<pwm2, timer, value_t, servo_0>;
    
    using pwm1 = PPMAdapter<pwm0_2, 2>;
    using adc1 = ADCAdapter<adc, 0>;
    using servo_1 = ::Servo<pwm1, adc1, timer, TermDev>;
    using pwm3 = PPMAsyncAdapter<pwm3s>;
    using esc1 = EscFsm<pwm3, timer, value_t, servo_0>;

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
        blinker1::init();
        blinker2::init();
        
        pwm0_2::init();
        pwm3s::init();
//        pwm4s::init();
//        pwm5s::init();
        
        adc::init();
        adc::mcu_adc_type::nsamples(4);
    }
    
    static inline void periodic() {
        servo::periodic();        
        servo_0::periodic();
        servo_1::periodic();
        esc0::periodic();
        esc1::periodic();
        
        adc::periodic();
    }
    
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        servo_0::ratePeriodic();
        servo_1::ratePeriodic();
        esc0::ratePeriodic();
        esc1::ratePeriodic();
        
        const auto oldState = mState;
        ++mPulseTick;
        ++mStateTick;
        ++mDebugTick;
        
        mDebugTick.on(debugTicks, []{
//            etl::outl<terminal>("ch0: "_pgm, servo_pa::value(0).toInt(), " ch3: "_pgm, servo_pa::value(3).toInt());
            servo_0::debug();
            servo_1::debug();
#ifndef NDEBUG
            if (xassert::on) {
                etl::outl<terminal>("assert: "_pgm, xassert::ab);
                xassert::on = false;
            }
#endif
        });
        
        mPulseTick.on(pulseTicks, []{
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

        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mStateTick.on(initTimeoutTicks, []{
                mState = State::ShowBus;
            });
            break;
        case State::ShowBus:
            mStateTick.on(showTimeoutTicks, []{
                mState = State::CheckConfig;
            });
            break;
        case State::CheckConfig:
            if (config1::isActive()) {
                mState = State::SearchChannel;
            }
            else {
                mState = State::Calibrate1;
            }
            break;
        case State::SearchChannel:
            if (const auto v = servo_pa::value(searchCh.toInt()); v) {
                if (v.toInt() > chThreshPosH.toInt()) {
                    mState = State::ShowChannel;
                }
                else {
                    ++searchCh;
                }
            }
            break;
        case State::ShowChannel:
            if (const auto v = servo_pa::value(searchCh.toInt()); v) {
                if (v.toInt() < chThreshPosL.toInt()) {
                    mState = State::ShowChannel2;
                }
            }
            break;
        case State::ShowChannel2:
            mStateTick.on(showTimeoutTicks, []{
                mState = State::Calibrate1;
            });
            break;
        case State::Calibrate1:
            if (servo_0::setupFinished()) {
                mState = State::Calibrate2;
//                mState = State::Run;
            }
            break;
        case State::Calibrate2:
            if (servo_1::setupFinished()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            const auto dir1 = servo_pa::value(searchCh + 1);
            servo_0::position(dir1);
            if (const auto d = servo_0::adiff(); d < 100) {
                const auto throttle = servo_pa::value(searchCh + 0);
                esc0::set(throttle);
            }
            else {
                esc0::off();
            }
            const auto dir2 = servo_pa::value(searchCh + 3);
            servo_1::position(dir2);
            if (const auto d = servo_1::adiff(); d < 100) {
                const auto throttle = servo_pa::value(searchCh + 2);
                esc0::set(throttle);
            }
            else {
                esc0::off();
            }
//            const auto thru = servo_pa::value(searchCh + 2);
//            setThru(thru);
//            const auto servoSpeed = servo_pa::value(searchCh + 3);
//            servo_0::speed(servoSpeed);
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S Ini"_pgm);
                servo_0::reset();
                break;
            case State::ShowBus:
                etl::outl<terminal>("S Sbu"_pgm);
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    blinker1::onePeriod(bcount_t{1});
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    blinker1::onePeriod(bcount_t{2});
                }
                break;
            case State::CheckConfig:
                etl::outl<terminal>("S CkC"_pgm);
                break;
            case State::SearchChannel:
                blinker1::blink(bcount_t{5});
                etl::outl<terminal>("S Sea"_pgm);
                break;
            case State::ShowChannel:
                etl::outl<terminal>("S Sch "_pgm, searchCh.toInt());
                blinker1::off();
//                data().channel() = channel_t{searchCh.toInt()};
//                data().change();
                break;
            case State::ShowChannel2:
                blinker1::steady();
                etl::outl<terminal>("S Sch2 "_pgm, searchCh.toInt());
                break;
            case State::Calibrate1:
                blinker1::off();
                blinker1::blink(bcount_t{4});
                etl::outl<terminal>("S Cal1"_pgm);
                servo_0::calibrate();
                break;
            case State::Calibrate2:
                blinker2::off();
                blinker2::blink(bcount_t{4});
                etl::outl<terminal>("S Cal2"_pgm);
                servo_1::calibrate();
                break;
            case State::Run:
                blinker1::off();
                etl::outl<terminal>("S Run"_pgm);
                servo_0::run();
                servo_1::run();
//                setEsc(ppm_t{PPM::ocMedium});
                break;
            }
        }
    }
private:
    inline static search_t searchCh{0};

    static inline State mState{State::Undefined};
    static inline External::Tick<timer> mStateTick{};
    static inline External::Tick<timer> mDebugTick{};
    static inline External::Tick<timer> mPulseTick{};
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

            etl::outl<terminal>("rc720_03"_pgm);
            
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

using app = Application<External::Bus::SBusSPort<devices>>;
int main() {
    devices::init();
    app::run(true);
        
//    scanner::run();
}

#ifndef NDEBUG
/*[[noreturn]] */ static inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    xassert::ab.clear();
    xassert::ab.insertAt(0, expr);
    etl::itoa(line, xassert::aline);
    xassert::ab.insertAt(20, xassert::aline);
    xassert::ab.insertAt(30, file);
    xassert::on = true;
}

template<typename String1, typename String2>
[[noreturn]] static inline void assertFunction(const String1& s1 [[maybe_unused]], const String2& s2 [[maybe_unused]], const unsigned int l [[maybe_unused]]) {
    assertOutput(s1, s2, l);
    
    while(true) {
        asm(";xxx");
        devices::assertPin::toggle();
    }
}
#endif
