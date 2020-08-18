#define NDEBUG

//#define USE_HOTT

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>

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
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart, SensorType,
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint8_t>& operator[](const AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
        inline void select(const AVKey) {}
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

// PA1: Voltage = AIN1
// PA2: Debug Pin
// PA3: Led
// PA4:
// PA5: current sense = AIN5
// PA6:
// PA7: Taster
// PB3:
// PB2: TXRX (Sensor)
// PB1: VN7003 = fet
// PB0: Buzzer = tca:wo0

using PortA = Port<A>;
using PortB = Port<B>;

using dbg1 = Pin<PortA, 2>; 
using led = Pin<PortA, 3>;
using button = Pin<PortA, 7>;
using fet = Pin<PortB, 1>;
using buzzer = Pin<PortB, 0>;

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 500_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 2>;

#ifdef USE_HOTT
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;
#else
using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<1, 5>>;

using buzzerPwm = PWM::DynamicPwm<tcaPosition>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

//class SensorChoice final : public Hott::Menu {
//public:
//    SensorChoice() : Menu(this, "Sensorauswahl"_pgm, &mRpm1, &mRpm2) {}
//private:
//    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm1{"Rpm1"_pgm, appData, Storage::AVKey::SoftStart, 2};
//    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mRpm2{"Rpm2"_pgm, appData, Storage::AVKey::TimeOut, 2};
//};

class RCMenu final : public Hott::Menu<Parameter::menuLines, true, 3> {
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
        PA::processKey([&](Hott::key_t k){
            mMenu = mMenu->processKey(k);
            clear();
//            processKey(k);
        });
//        if (auto k = PA::key(); k != Hott::key_t::nokey) {
//            processKey(k);
//        }
        mMenu->textTo(PA::text());
    }
//    inline static void processKey(Hott::key_t key) {
//        assert(mMenu);
//        if (auto m = mMenu->processKey(key); m != mMenu) {
//            mMenu = m;
//            clear();
//        }
//    }
private:
    inline static void clear() {
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::Menu<PA::menuLines>* mMenu = &mTopMenu;
};

#ifdef USE_HOTT
using menu = HottMenu<sensor, RCMenu>;
#endif

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    dbg1::template dir<Output>();
    led::template dir<Output>();
    button::template dir<Input>();
    fet::template dir<Output>();

    portmux::init();
    eeprom::init();
    systemTimer::init();
    adcController::init();
#ifdef USE_HOTT
    sensor::init();
    menu::init();
#else
    terminalDevice::init<AVR::BaudRate<9600>>();
#endif
    
    buzzerPwm::init();
    buzzerPwm::frequency(4000_Hz);
    buzzerPwm::template duty<Meta::List<PWM::WO<0>>>(1000);
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    uint8_t counter = 0;
    
    bool eepSave = false;
    while(true) {
        eepSave |= eeprom::saveIfNeeded();
#ifdef USE_HOTT
        sensor::periodic();
        menu::periodic();
#else
        terminalDevice::periodic();
#endif
        systemTimer::periodic([&]{
#ifdef USE_HOTT
            sensor::ratePeriodic();
#endif
            dbg1::toggle();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ++counter;
                    led::toggle();
                    if ((counter % 2) == 0) {
#ifndef USE_HOTT
                        etl::outl<terminal>("test00"_pgm);
#endif
                        buzzerPwm::template on<Meta::List<PWM::WO<0>>>();
                    }
                    else {
                        buzzerPwm::template off<Meta::List<PWM::WO<0>>>();
                    }
                }
            });
            appData.expire();
        });
    }
}

