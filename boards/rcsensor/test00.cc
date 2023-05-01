#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/capture.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>

#include <external/hott/experimental/adapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/menu.h>

#include <external/solutions/gps.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>

#include <external/bluetooth/qtrobo.h>
#include <external/bluetooth/roboremo.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

// curiosity
using led = Pin<Port<F>, 5>; 

// debug-pins
using debug0 = Pin<Port<F>, 2>; 
using debug1 = Pin<Port<A>, 0>; 

// nicht alle pins wegen channels wählbar
using rpmPin0 = Pin<Port<E>, 0>; 
using rpmPin1 = Pin<Port<E>, 1>; 
using rpmPin2 = Pin<Port<C>, 0>; 

using ppmPin0 = Pin<Port<C>, 1>; 
using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>;
using usart3Position = Portmux::Position<Component::Usart<3>, Portmux::Default>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, usart3Position, tcaPosition>>;

//using sensorQuery = Hott::Experimental::SensorQuery<usart0Position, AVR::Usart, AVR::BaudRate<19200>>;

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter<>, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

//using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
//using rcUsart = AVR::Usart<usart1Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using ui = External::QtRobo::ProtocollAdapter<0>;
using uiUsart = AVR::Usart<usart1Position, ui, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using gps = External::GPS::GpsProtocollAdapter<0, External::GPS::VTG>;
using gpsUsart = AVR::Usart<usart2Position, gps, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

namespace {
    constexpr uint8_t menuLines = 4;
    constexpr auto dt = 2_ms;
    constexpr auto fRtc = 512_Hz;
}

using systemTimer = SystemTimer<Component::Pit<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sensor = Hott::Experimental::Sensor<usart3Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;

using rpm0 = Capture<Component::Tcb<0>>;
using rpm1 = Capture<Component::Tcb<1>>;
using rpm2 = Capture<Component::Tcb<2>>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15>>;
using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t, std::ratio<121,21>>; // todo: richtiger scale faktor
using currentConverter = Hott::Units::Converter<adc, Hott::Units::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor

namespace Storage {
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart, SensorType, Magic0, Magic1, _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        using value_type = etl::uint_NaN<uint8_t>;
        value_type& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<value_type, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
}

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();


class RCMenu final : public Hott::Menu<menuLines> {
public:
    RCMenu() : Menu<menuLines>(this, "WM Sensor 1.0"_pgm, &mSoft, &mTimeout, &mType) {}
private:
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mSoft{"Start"_pgm, appData, Storage::AVKey::SoftStart, 2};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mTimeout{"TimeOut"_pgm, appData, Storage::AVKey::TimeOut, 10};
    Hott::TextWithValue<Storage::AVKey, Storage::ApplData> mType{"Type"_pgm, appData, Storage::AVKey::SensorType, 2};
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
        });
        mMenu->textTo(PA::text());
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

struct Measurements {
    static inline void periodic() {
        etl::circular_call(part0, part1, part2, part3);
    }
    static inline void tick() {
        c += ((uint32_t)last_current.value * 2); // 200ms
    }
private:
    inline static auto sensorData = Hott::Experimental::Adapter<sensor::binaryMessage_type>(sensor::data());

    inline static Hott::Units::battery_voltage_t voltage_min{std::numeric_limits<Hott::Units::battery_voltage_t::value_type>::max()};
    inline static Hott::Units::battery_voltage_t last_voltage;
    inline static Hott::Units::current_t current_max;
    inline static Hott::Units::current_t last_current;
    inline static External::Units::celsius<uint16_t, std::ratio<1, 1>> temp_max;

    inline static uint32_t c = 0;
    
    static inline void part0() {
        auto v = battVoltageConverter::convert(adcController::value(adcController::index_type{0}));
        last_voltage = v;
        sensorData.voltage(v);
        if (v > Hott::Units::battery_voltage_t{20}) {
            voltage_min = std::min(voltage_min, v);
            sensorData.voltageMin(voltage_min);
        }
        
    }
    static inline void part1() {
        auto c = currentConverter::convert(adcController::value(adcController::index_type{1}));
        last_current = c;
        sensorData.current(c);
        current_max = std::max(current_max, c);
        sensorData.currentMax(current_max);
        
    }
    static inline void part2() {
        auto t = sigrow::adcValueToTemperature(adcController::value(adcController::index_type{2}));
        sensorData.temp(t);
        temp_max = std::max(temp_max, t);
        sensorData.tempMax(temp_max);
    }
    static inline void part3() {
        sensorData.capRaw(c / 3600);
    }
};

using measurements = Measurements;


using rpm0_channel = Event::Channel<4, Event::Generators::Pin<rpmPin0>>; 
using rpm1_channel = Event::Channel<5, Event::Generators::Pin<rpmPin1>>; 
using rpm2_channel = Event::Channel<2, Event::Generators::Pin<rpmPin2>>; 
using ppm0_channel = Event::Channel<3, Event::Generators::Pin<ppmPin0>>; 
using rpm0_user = Event::Route<rpm0_channel, Event::Users::Tcb<0>>;
using rpm1_user = Event::Route<rpm1_channel, Event::Users::Tcb<1>>;
using rpm2_user = Event::Route<rpm2_channel, Event::Users::Tcb<2>>;
using ppm0_user = Event::Route<ppm0_channel, Event::Users::Tcb<3>>;

using evrouter = Event::Router<Event::Channels<rpm0_channel, rpm1_channel, rpm2_channel, ppm0_channel>, Event::Routes<rpm0_user, rpm1_user, rpm2_user, ppm0_user>>;

int main() {
    portmux::init();
    evrouter::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    eeprom::init();
    
    [[maybe_unused]] bool changed = false;
    
    {
        constexpr auto m0 = Storage::ApplData::value_type{43};
        constexpr auto m1 = Storage::ApplData::value_type{44};
        if (!((appData[Storage::AVKey::Magic0] == m0) && (appData[Storage::AVKey::Magic1] == m1))) {
            appData[Storage::AVKey::Magic0] = m0;
            appData[Storage::AVKey::Magic1] = m1;
            appData.change();
            changed  = true;
        }
    }
    
    
    terminalDevice::init<BaudRate<9600>>();
    gpsUsart::init<BaudRate<9600>>();
//    rcUsart::init<BaudRate<115200>>();
    uiUsart::init<BaudRate<9600>>();
    
    systemTimer::init();
    sensor::init();
    menu::init();
    
    rpm0::init();
    rpm1::init();
    rpm2::init();
    
    adcController::init();
    
    led::template dir<Output>();     
    debug0::template dir<Output>();     
    debug1::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    {
        etl::Scoped<etl::EnableInterrupt<>> ei;

        bool eepSave{false};
        
        while(true) {
            debug0::toggle();
            
            eeprom::saveIfNeeded([&]{
                eepSave = true;
//                fsm::load();                
            });
            
            
            terminalDevice::periodic();
//            rcUsart::periodic();
            uiUsart::periodic();
            gpsUsart::periodic();
            sensor::periodic();
            menu::periodic();
            
            systemTimer::periodic([&]{
                debug1::toggle();
                sensor::ratePeriodic();
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        led::toggle();
                        etl::outl<terminal>("test00"_pgm);
                        if (eepSave) {
                            etl::outl<terminal>("es"_pgm);
                            eepSave = false;
                        }
                        if (changed) {
                            etl::outl<terminal>("ni"_pgm);
                        }
                    }
                });
            });
        }
    }
}

