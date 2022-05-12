#define NDEBUG

//#define LOG_OUTPUT

#ifndef NDEBUG
//static unsigned int assertKey{1234};
#endif
  
#include <mcu/avr.h>
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

#include <etl/output.h>
#include <etl/meta.h>

#include "devices.h"
#include "busdevs.h"
#include "version.h"

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


template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    
    using bus_type = BusDevs::bus_type;
    using devs = BusDevs::devs;
    using nvm = devs::eeprom;
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
                                Init, Calibrate1, Calibrate2, 
                                SetZero, SaveZero,
                                InitRun,
                                Run};
    
    static constexpr External::Tick<timer> startupTicks{100_ms};
    static constexpr External::Tick<timer> initTimeoutTicks{300_ms};
    static constexpr auto showTimeout = 1000_ms;
    static constexpr External::Tick<timer> showTimeoutTicks{showTimeout};
    static constexpr External::Tick<timer> eepromTicks{1000_ms};
    static constexpr External::Tick<timer> showTimeoutTicksLong{2000_ms};
    
    static constexpr External::Tick<timer> debugTicks{500_ms};
    static constexpr External::Tick<timer> pulseTicks{20_ms};

    static inline constexpr value_t chThreshPosH = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshPosL = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};

//    static inline constexpr value_t chThreshNegH = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 4};
//    static inline constexpr value_t chThreshNegL = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 6};

    using blinker1 = External::Blinker2<typename devs::led1, timer, 100_ms, showTimeout>;
    using blinker2 = External::Blinker2<typename devs::led2, timer, 100_ms, showTimeout>;
    using bcount_t = blinker1::count_type;

    using servo_0 = BusDevs::servo_0;
    using servo_1 = BusDevs::servo_1;
    
    using esc0 = BusDevs::esc0;
    using esc1 = BusDevs::esc1;

    using pwm4 = BusDevs::pwm4;
    using pwm5 = BusDevs::pwm5;
    
    static_assert(std::is_same_v<typename pwm4::ranged_type, typename pwm5::ranged_type>);
    using pwm_t = pwm4::ranged_type;
    
    static inline void init(const bool inverted) {
        BusDevs::init(inverted);
        
        searchCh = search_t{appData.channel()};
        
        if (const auto z = appData.userZero1(); (z < 4096) && (z > -4096)) {
            servo_0::zero() = appData.userZero1();
        }
        if (const auto z = appData.userZero2(); (z < 4096) && (z > -4096)) {
            servo_1::zero() = appData.userZero2();
        }
        
        blinker1::init();
        blinker2::init();
        
        config1::init(); // before pwm4/5
        config2::init();
    }
    
    static inline void periodic() {
        nvm::saveIfNeeded([&]{
            etl::outl<terminal>("ep s"_pgm);
        });
        
        servo::periodic();        
        servo_0::periodic();
        servo_1::periodic();
        esc0::periodic();
        esc1::periodic();
        adc::periodic();
        sensor::periodic();
    }
    
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        servo_0::ratePeriodic();
        servo_1::ratePeriodic();
        esc0::ratePeriodic();
        esc1::ratePeriodic();
        sensor::ratePeriodic();
        
        blinker1::ratePeriodic();
        blinker2::ratePeriodic();
        
        const auto oldState = mState;
        ++mPulseTick;
        ++mStateTick;
        ++mDebugTick;
        
        mDebugTick.on(debugTicks, []{
//            etl::outl<terminal>("ch0: "_pgm, servo_pa::value(0).toInt(), " ch3: "_pgm, servo_pa::value(3).toInt());
//            etl::outl<terminal>("p: "_pgm, sensor_pa::requests());
//            etl::outl<terminal>("ad0: "_pgm, servo_0::adiff());
//            etl::outl<terminal>("z0: "_pgm, servo_0::zero());
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
            BusDevs::onPulse();
        });
        (++mEepromTick).on(eepromTicks, []{
            appData.expire();
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
                inConfigMode = true;
            }
            else {
                inConfigMode = false;
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
            }
            break;
        case State::Calibrate2:
            if (servo_1::setupFinished()) {
                mState = State::InitRun;
            }
            break;
        case State::InitRun:
            mStateTick.on(startupTicks, []{
                if (inConfigMode) {
                    mState = State::SetZero; 
                }
                else {
                    mState = State::Run;
                }
            });
            break;
        case State::SetZero:
        {
            constexpr value_t neutral{(value_t::Upper + value_t::Lower) / 2};
            if (const auto dir = servo_pa::value(searchCh + 4); dir) {
                servo_0::zero(dir);
                servo_0::position(neutral);
            }
            if (const auto dir = servo_pa::value(searchCh + 5); dir) {
                servo_1::zero(dir);
                servo_1::position(neutral);
            }
            if (const auto v = servo_pa::value(searchCh.toInt()); v) {
                if (v.toInt() > chThreshPosH.toInt()) {
                    mState = State::SaveZero;
                }
            }
        }
            break;
        case State::SaveZero:
            mStateTick.on(showTimeoutTicksLong, []{
                mState = State::Run; 
            });
            break;
        case State::Run:
            const auto dir1 = servo_pa::value(searchCh + 1);
            servo_0::position(dir1);
            if (const auto d = servo_0::adiff(); d < offDiff0) {
                const auto throttle = servo_pa::value(searchCh + 0);
                esc0::set(throttle);
            }
            else {
                esc0::off();
            }
            const auto dir2 = servo_pa::value(searchCh + 3);
            servo_1::position(dir2);
            if (const auto d = servo_1::adiff(); d < offDiff1) {
                const auto throttle = servo_pa::value(searchCh + 2);
                esc1::set(throttle);
            }
            else {
                esc1::off();
            }
            if (searchCh <= (servo_pa::channel_t::Upper - 4)) {
                if (const auto thru = servo_pa::value(searchCh + 4)) {
                    const auto thru_p = AVR::Pgm::scaleTo<pwm_t>(thru.toRanged());
                    pwm4::set(thru_p);
                }
            }
            if (searchCh <= (servo_pa::channel_t::Upper - 5)) {
                if (const auto thru = servo_pa::value(searchCh + 5)) {
                    const auto thru_p = AVR::Pgm::scaleTo<pwm_t>(thru.toRanged());
                    pwm5::set(thru_p);
                }
            }
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
                servo_1::reset();
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
                etl::outl<terminal>("S Sea"_pgm);
                blinker1::blink(bcount_t{3});
                break;
            case State::ShowChannel:
                etl::outl<terminal>("S Sch "_pgm, searchCh.toInt());
                blinker1::off();
                blinker2::steady();
                devs::eeprom::data().channel(searchCh);
                devs::eeprom::data().change();
                break;
            case State::ShowChannel2:
                etl::outl<terminal>("S Sch2 "_pgm, searchCh.toInt());
                blinker1::steady();
                break;
            case State::Calibrate1:
                etl::outl<terminal>("S Cal1"_pgm);
                blinker1::off();
                blinker2::off();
                blinker1::blink(bcount_t{2});
                servo_0::calibrate();
                break;
            case State::Calibrate2:
                etl::outl<terminal>("S Cal2"_pgm);
                blinker1::off();
                blinker2::off();
                blinker2::blink(bcount_t{2});
                servo_1::calibrate();
                break;
            case State::InitRun:
                etl::outl<terminal>("S InitRun"_pgm);
                blinker1::off();
                blinker2::off();
                devs::pwm4::init();
                devs::pwm5::init();
                break;
            case State::SetZero:
                etl::outl<terminal>("S SetZ"_pgm);
                blinker1::blink(bcount_t{3});
                blinker2::blink(bcount_t{3});
                servo_0::run();
                servo_1::run();
                break;
            case State::SaveZero:
                etl::outl<terminal>("S SetSZ"_pgm);
                blinker1::blink(bcount_t{4});
                blinker2::blink(bcount_t{4});
                devs::eeprom::data().userZero1() = servo_0::zero();
                devs::eeprom::data().userZero2() = servo_1::zero();
                devs::eeprom::data().change();
                break;
            case State::Run:
                etl::outl<terminal>("S Run"_pgm);
                offDiff0 = servo_0::span() / 16;
                offDiff1 = servo_1::span() / 16;
                blinker1::off();
                blinker2::off();
                blinker1::blink(bcount_t{1});
                blinker2::blink(bcount_t{1});
                servo_0::run();
                servo_1::run();
                break;
            }
        }
    }
private:
    static inline bool inConfigMode{false};
    static inline uint16_t offDiff0{4000};
    static inline uint16_t offDiff1{4000};
    
    static inline auto& appData = nvm::data();

    inline static search_t searchCh{0};

    static inline State mState{State::Undefined};
    static inline External::Tick<timer> mStateTick{};
    static inline External::Tick<timer> mDebugTick{};
    static inline External::Tick<timer> mPulseTick{};
    static inline External::Tick<timer> mEepromTick{};
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

int main() {
    scanner::run();
}

#ifndef NDEBUG
/*[[noreturn]] */ static inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
//    xassert::ab.clear();
//    xassert::ab.insertAt(0, expr);
//    etl::itoa(line, xassert::aline);
//    xassert::ab.insertAt(20, xassert::aline);
//    xassert::ab.insertAt(30, file);
//    xassert::on = true;
}

template<typename String1, typename String2>
[[noreturn]] static inline void assertFunction(const String1& s1 [[maybe_unused]], const String2& s2 [[maybe_unused]], const unsigned int l [[maybe_unused]]) {
//    assertOutput(s1, s2, l);
    
    while(true) {
        asm(";xxx");
        devices::assertPin::toggle();
    }
}
#endif
