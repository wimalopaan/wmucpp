//#define NDEBUG

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
#include <external/solutions/rc/multi.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

#include "devices.h"
#include "busdevs_multi.h"
#include "version.h"

#include "storage.h"
#include "fsm.h"

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

template<typename Devs, typename FsmList>
struct GFSM;

template<typename Devs, typename... Fsms>
struct GFSM<Devs, Meta::List<Fsms...>> {
    using bus_type = Devs::bus_type;
    using devs = bus_type::devs;
    
    using Servo = Devs::servo;
    using servo_pa = Devs::servo_pa;
    
    using ch_t = servo_pa::channel_t;
    
    enum class State : uint8_t {Undefined, StartShowBus, ShowBus, StartWait, SearchChannel, AfterSearch, InitRun, Run, 
                                ShowAddress, ShowAddressWait, LearnTimeout,
                                Test1};
    
    using Timer = Devs::systemTimer;
    using SW = Devs::bus_switch;
    using Term = Devs::terminal;
    using NVM = Devs::eeprom;
    using RELD = Devs::reloader;
//    using evrouter = Devs::evrouter;
    
    static constexpr auto intervall = Timer::intervall;
    
    inline static constexpr auto learnTimeout = 4000_ms;
    inline static constexpr auto scanTimeout = 50_ms;
    
    static_assert(learnTimeout > (scanTimeout * (ch_t::Upper + 1) * 2), "");
    
    static constexpr External::Tick<Timer> learnTimeoutTicks{learnTimeout};
    static constexpr External::Tick<Timer> scanTimeoutTicks{scanTimeout};
    static constexpr External::Tick<Timer> waitTimeoutTicks{3000_ms};
    static constexpr External::Tick<Timer> reloadTimeoutTicks{20_ms};
    static constexpr External::Tick<Timer> signalTimeoutTicks{500_ms};
    static constexpr External::Tick<Timer> eepromTimeout{1000_ms};
    
    using blinker = External::SimpleBlinker<typename devs::led1, Timer, 300_ms>;
    
    inline static auto& data() {
        return NVM::data();
    }
    
    static inline void init(const bool inverted) {
        SW::init();
        NVM::init();
        if (data().magic() != 42) {
            data().magic() = 42;
            data().clear();
            data().change();
            etl::outl<Term>("e init"_pgm);
        }
        
//        wdt::init<ccp>();        
//        reset::noWatchDog([]{
//            uninitialzed::reset();
//        });
//        reset::onWatchDog([]{
//            uninitialzed::counter = uninitialzed::counter + 1;        
//        });
        
        blinker::init();
        
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            Servo::template init<BaudRate<115200>, HalfDuplex>();
            Servo::template txEnable<false>();
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            Servo::template init<AVR::BaudRate<100000>, HalfDuplex, true, 1>(); // 8E2
            Servo::template txEnable<false>();
            if (inverted) {
                Servo::txInvert(true);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
    }
    
    static inline void periodic() {
        Servo::periodic();
        NVM::saveIfNeeded([&]{
            etl::outl<Term>("ep s"_pgm);
        });
        switch(mState) {
        case State::Run:
            (Fsms::periodic(), ...);
            break;
        case State::Undefined:
        case State::StartShowBus:
        case State::ShowBus:
        case State::StartWait:
        case State::SearchChannel:
        case State::AfterSearch:
        case State::InitRun:
        case State::ShowAddress:
        case State::ShowAddressWait:
        case State::LearnTimeout:
        case State::Test1:
        default:
            break;
        }
    }
    
    static inline void test(const auto) {}
    
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
//        wdt::reset();
        const auto oldState = mState;
        blinker::ratePeriodic();
        ++stateTicks;
        ++mEepromTicks;

        switch(mState) {
        case State::Undefined:
            mState = State::StartShowBus;
            break;
        case State::StartShowBus:
            stateTicks.on(signalTimeoutTicks, []{
                mState = State::ShowBus;
            });
            break;
        case State::ShowBus:
            stateTicks.on(waitTimeoutTicks, []{
                mState = State::StartWait;
            });
            break;
        case State::StartWait:
            stateTicks.on(waitTimeoutTicks, []{
                mState = State::SearchChannel;
            });
            break;
        case State::SearchChannel:
            if (search()) {
                mState = State::AfterSearch;
            }
            stateTicks.on(learnTimeoutTicks, []{
                mState = State::LearnTimeout;
            });
            break;
        case State::AfterSearch:
            stateTicks.on(signalTimeoutTicks, []{
                mState = State::ShowAddress;
            });
            break;
        case State::LearnTimeout:
//            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(typename RCSwitch::addr_t{NVM::data().address().toInt()});
            }
            mState = State::InitRun;
            break;
        case State::ShowAddress:
//            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel(), " adr: "_pgm, NVM::data().address().toInt());
            blinker::blink(NVM::data().address().toInt() + 1);
            mState = State::ShowAddressWait;
            break;
        case State::ShowAddressWait:
            if (!blinker::isActive()) {
                mState = State::InitRun;
            }
            break;
        case State::Run:
            SW::ratePeriodic();
            stateTicks.on(reloadTimeoutTicks, []{
                RELD::reload();
            });
            if (SW::receivedControl()) {
                blinker::steady();                
            }
            else {
                blinker::off();                
            }
            mEepromTicks.on(eepromTimeout, []{
                NVM::data().expire();
            });
            break;
        case State::InitRun:
            (Fsms::init(), ...);
            RELD::init();
            mState = State::Run;
            break;
        case State::Test1:
            if (!SW::receivedControl()) {
                mState = State::Run;
            }
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Run:
                break;
            case State::Undefined:
                break;
            case State::StartShowBus:
                blinker::off();
                break;
            case State::ShowBus:
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    blinker::blink(1);
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    blinker::blink(2);
                }
                break;
            case State::StartWait:
                blinker::steady();
                etl::outl<Term>("swait"_pgm);
                break;
            case State::SearchChannel:
                blinker::off();
                etl::outl<Term>("search"_pgm);
                break;
            case State::AfterSearch:
            case State::InitRun:
            case State::ShowAddress:
            case State::ShowAddressWait:
            case State::LearnTimeout:
            case State::Test1:
            default:
                break;
            }
        }
    }
private:
    using protocol_t = Devs::BusParam::proto_type;
    using addr_t = RCSwitch::addr_t;
    
    static inline bool search() {
        if (const auto lc = servo_pa::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
            if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= addr_t::Upper)) {
                const uint8_t addr = pv - 1;
                SW::channel(learnChannel.toRangedNaN());
                SW::address(addr_t(addr));
                NVM::data().channel() = learnChannel;
                NVM::data().address() = addr;
                NVM::data().change();
                return true;
            }
        }   
#ifdef LEARN_DOWN
        --learnChannel;
#else
        ++learnChannel;
#endif
        return false;
    }
#ifdef LEARN_DOWN
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{ch_t::Upper};
#else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
#endif
    static inline State mState{State::Undefined};
    inline static External::Tick<Timer> stateTicks;
    inline static External::Tick<Timer> mEepromTicks;
};


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

    using servo = BusDevs::servo;
    using servo_pa = BusDevs::servo_pa;
    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    using value_t = servo_pa::value_type;
    
    using sensor = BusDevs::sensor;
    using sensor_pa = sensor::ProtocollAdapter;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    enum class State : uint8_t {Undefined, 
                                ShowBus, Init,
                                Run};
    
    static constexpr External::Tick<timer> startupTicks{100_ms};
    static constexpr External::Tick<timer> initTimeoutTicks{300_ms};
    static constexpr auto showTimeout = 1000_ms;
    static constexpr External::Tick<timer> showTimeoutTicks{showTimeout};
    static constexpr External::Tick<timer> eepromTicks{1000_ms};
    static constexpr External::Tick<timer> showTimeoutTicksLong{2000_ms};
    
    static constexpr External::Tick<timer> debugTicks{500_ms};
    static constexpr External::Tick<timer> pulseTicks{20_ms};


    using blinker1 = External::Blinker2<typename devs::led1, timer, 100_ms, showTimeout>;
    using blinker2 = External::Blinker2<typename devs::led2, timer, 100_ms, showTimeout>;
    using bcount_t = blinker1::count_type;


    using pwm4 = BusDevs::pwm4;
    using pwm5 = BusDevs::pwm5;
    
    static_assert(std::is_same_v<typename pwm4::ranged_type, typename pwm5::ranged_type>);
    using pwm_t = pwm4::ranged_type;
    
    static inline void init(const bool inverted) {
        BusDevs::init(inverted);
        
        searchCh = search_t{appData.channel()};
        
        blinker1::init();
        blinker2::init();
        
    }
    
    static inline void periodic() {
        nvm::saveIfNeeded([&]{
            etl::outl<terminal>("ep s"_pgm);
        });
        
        servo::periodic();        
        adc::periodic();
        sensor::periodic();
    }
    
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        sensor::ratePeriodic();
        
        blinker1::ratePeriodic();
        blinker2::ratePeriodic();
        
        const auto oldState = mState;
        ++mPulseTick;
        ++mStateTick;
        ++mDebugTick;
        
        mDebugTick.on(debugTicks, []{
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
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S Ini"_pgm);
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
            using gfsm = GFSM<devs, typename devs::fsms>;
            
            gfsm::init(inverted);

            etl::outl<terminal>("multiAdapter_01"_pgm);
            
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
