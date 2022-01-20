//#define NDEBUG

#ifndef NDEBUG
//static unsigned int assertKey{1234};
#endif
  
#define LEARN_DOWN // start at highest channel number downwards
#define START_AT_CHANNEL_16

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
    
    using servo = Devs::servo;
    using servo_pa = Devs::servo_pa;
    
    using sensor = Devs::sensor;
    
    using ch_t = servo_pa::channel_t;
    
    enum class State : uint8_t {Undefined, StartShowBus, ShowBus, StartWait, SearchChannel, AfterSearch, InitRun, Run, 
                                ShowAddress, ShowAddressWait, LearnTimeout};
    
    using Timer = Devs::systemTimer;
    using SW = Devs::bus_switch;
    using Term = Devs::terminal;
    using NVM = Devs::eeprom;
    using RELD = Devs::reloader;
    
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
    
    using blinker1 = External::SimpleBlinker<typename devs::led1, Timer, 300_ms>;
    using blinker2 = External::SimpleBlinker<typename devs::led2, Timer, 300_ms>;
    
    inline static auto& data() {
        return NVM::data();
    }
    
    static inline void init(const bool inverted) {
        Devs::init(inverted);        
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
        
        blinker1::init();
        blinker2::init();
    }
    
    static inline void periodic() {
        sensor::periodic();
        servo::periodic();
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
        default:
            break;
        }
    }
    
    static inline void test(const auto) {}
    
    static inline void ratePeriodic() {
        sensor::ratePeriodic();
        servo_pa::ratePeriodic();
//        wdt::reset();
        const auto oldState = mState;
        blinker1::ratePeriodic();
        blinker2::ratePeriodic();
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
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(typename RCSwitch::addr_t{NVM::data().address().toInt()});
            }
            mState = State::InitRun;
            break;
        case State::ShowAddress:
            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            blinker1::blink(NVM::data().address().toInt() + 1);
            mState = State::ShowAddressWait;
            break;
        case State::ShowAddressWait:
            if (!blinker1::isActive()) {
                mState = State::InitRun;
            }
            break;
        case State::Run:
            SW::ratePeriodic();
            stateTicks.on(reloadTimeoutTicks, []{
                RELD::reload();
            });
            if (SW::receivedControl()) {
                blinker1::steady();                
            }
            else {
                blinker1::off();                
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
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Run:
                blinker2::steady();
                break;
            case State::Undefined:
                break;
            case State::StartShowBus:
                blinker1::off();
                blinker2::off();
                break;
            case State::ShowBus:
                blinker2::off();
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    blinker1::blink(1);
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    blinker1::blink(2);
                }
                break;
            case State::StartWait:
                blinker1::steady();
                etl::outl<Term>("swait"_pgm);
                break;
            case State::SearchChannel:
                blinker1::off();
                blinker2::blink(3);
                etl::outl<Term>("search"_pgm);
                break;
            case State::AfterSearch:
            case State::InitRun:
            case State::ShowAddress:
                blinker2::steady();
                break;
            case State::ShowAddressWait:
            case State::LearnTimeout:
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
# ifdef START_AT_CHANNEL_16
        if (learnChannel == 0) {
            learnChannel = etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper>{15};
        }
        else {
            --learnChannel;
        }
# else
        --learnChannel;
# endif
#else
        ++learnChannel;
#endif
        return false;
    }
#ifdef LEARN_DOWN
# ifdef START_AT_CHANNEL_16
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{15};
# else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{ch_t::Upper};
# endif
#else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
#endif
    static inline State mState{State::Undefined};
    inline static External::Tick<Timer> stateTicks;
    inline static External::Tick<Timer> mEepromTicks;
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
        devices::assertPin::toggle();
    }
}
#endif
