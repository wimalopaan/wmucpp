#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/hott.h>
#include <external/hott/menu.h>

#include <std/chrono>

#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

struct Storage final {
    Storage() = delete;
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart,
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using PortA = Port<A>;
using dbg1 = Pin<PortA, 5>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;

//using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
//using terminal = etl::basic_ostream<terminalDevice>;

namespace Parameter {
    constexpr uint8_t menuLines = 7;
    constexpr auto dt = 2000_us;
    constexpr auto fRtc = 500_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 2>;

using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg_1, Hott::VarTextMsg<Parameter::menuLines>, systemTimer>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

//class SensorChoice final : public Hott::Menu {
//public:
//    SensorChoice() : Menu(this, "Sensorauswahl"_pgm, &mRpm1, &mRpm2) {}
//private:
//    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm1{"Rpm1"_pgm, appData, Storage::AVKey::SoftStart, 2};
//    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm2{"Rpm2"_pgm, appData, Storage::AVKey::TimeOut, 2};
//};

class RCMenu final : public Hott::Menu<Parameter::menuLines> {
public:
    RCMenu() : Menu(this, "OnOff 1.0"_pgm, &mRpm1, &mRpm2) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm1{"Rpm1"_pgm, appData, Storage::AVKey::SoftStart, 2};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm2{"Rpm2"_pgm, appData, Storage::AVKey::TimeOut, 2};
//    SensorChoice mSensorChoice;
};

template<typename PA, typename TopMenu>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void init() {
        clear();
    }
    inline static void periodic() {
        if (auto k = PA::key(); k != Hott::key_t::nokey) {
            processKey(k);
        }
        mMenu->textTo(PA::text());
    }
    inline static void processKey(Hott::key_t key) {
        assert(mMenu);
        if (auto m = mMenu->processKey(key); m != mMenu) {
            mMenu = m;
            clear();
        }
    }
private:
    inline static void clear() {
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::Menu<PA::menuLines>* mMenu = &mTopMenu;
};

using menu = HottMenu<sensor, RCMenu>;


int main() {
    eeprom::init();
    
    dbg1::template dir<Output>();
    portmux::init();

    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();

    sensor::init();
    
    adcController::init();
    
    menu::init();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    bool eepSave = false;
    
    while(true) {
        sensor::periodic();

        eepSave |= eeprom::saveIfNeeded();
        
        menu::periodic();
        
        systemTimer::periodic([&]{
            sensor::ratePeriodic();
            dbg1::toggle();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
//                    etl::outl<terminal>("test00"_pgm);
                }
                
            });
            appData.expire();
        });
    }
}

