// todo:
// temperatur-Kalibrierung / Kennlinie
// ibus protokoll: verbrauchte Kapazität
// Abbruchtöne länger
// calibrierung über eeprom (offset / steigung) für spannung / strom
// hott-sensor-type (eeprom)

#define NDEBUG

#define USE_SPORT
//#define USE_HOTT
//#define USE_IBUS
//#define FS_I6S
//#define USE_DAISY
//#define USE_EEPROM

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/reset.h>
#include <mcu/internals/watchdog.h>
#include <mcu/common/uninitialized.h>

#include <mcu/pgm/pgmarray.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>
#include <external/ibus/ibus.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/units/music.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/analogsensor.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SPORT
    constexpr auto fRtc = 2000_Hz;
#endif
    
    constexpr uint16_t R1vd = 10'000;
    constexpr uint16_t R2vd = 1'000;

    constexpr uint16_t Rc = 1'000;
    constexpr uint16_t Kc = 16'450;
}

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

#ifdef USE_EEPROM
using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();
#endif

using reset = Reset<>;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using wdt   = WatchDog<systemTimer::intervall>;
using uninitialzed = Uninitialized<>;

// PA1: Voltage = AIN1
// PA2: Debug Pin
// PA3: Led
// PA4:
// PA5: current sense = AIN5
// PA6: (daisy)
// PA7: Taster
// PB3:
// PB2: TXRX (Sensor)
// PB1: VN7003 = fet
// PB0: Buzzer = tca:wo0

using PortA = Port<A>;
using PortB = Port<B>;

#ifdef USE_DAISY
using daisyChain= Pin<PortA, 7>; 
#endif

using dbg       = Pin<PortA, 2>; 
using ledPin    = ActiveHigh<Pin<PortB, 1>, Output>;
using led       = External::Blinker<ledPin, systemTimer::intervall, 30_ms, 1000_ms>;
using buttonPin = Pin<PortA, 5>;
using button    = External::Button<ActiveLow<buttonPin, Input>, systemTimer, External::Tick<systemTimer>{100_ms}, External::Tick<systemTimer>{3000_ms}>;
using fet       = ActiveHigh<Pin<PortA, 3>, Output>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<1, 4, 0x1e>>; // 1e = temp

#ifdef USE_SPORT
using rxPin = Pin<Port<B>, 2>;
using txPin = rxPin;
template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcd<0>, 
PA, AVR::BaudRate<57600>,
AVR::ReceiveQueueLength<0>,
AVR::SendQueueLength<64>,
etl::NamedFlag<true>
>;

using acs770 = External::AnalogSensor<adcController, 1, std::ratio<1,2>, std::ratio<40,1000>, std::ratio<10,1>>;
using vdiv = External::AnalogSensor<adcController, 0, std::ratio<0,1>, std::ratio<1,10>, std::ratio<100,1>>;

template<typename Sensor>
struct CurrentProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Current;
    inline static uint32_t value() {
        return Sensor::value();
    }
};

template<typename Sensor>
struct VoltageProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Voltage;
    inline static uint32_t value() {
        return Sensor::value();
    }
};

using vProv1 = CurrentProvider<acs770>;
using vProv2 = VoltageProvider<vdiv>;

using sensor = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
                                       Meta::List<vProv1, vProv2>>;

using isrRegistrar = IsrRegistrar<sensor::uart::StartBitHandler, sensor::uart::BitHandler>;

#endif

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

#ifndef USE_SPORT
using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
#endif
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using sleep = Sleep<>;

#ifdef USE_IBUS
template<typename ADC, uint8_t Channel>
struct VoltageProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
//    index_t::_;
    inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
    inline static constexpr void init() {}
    
    using battVoltageConverter = Hott::Units::Converter<adc, IBus::battery_voltage_t, std::ratio<11000,1000>>; // Voltage divider 10k/1k 
//    using battVoltageConverter = Hott::Units::Converter<adc, IBus::battery_voltage_t, std::ratio<121,21>>; // Voltage divider 10k/1k 
    
    inline static constexpr uint16_t value() {
        return battVoltageConverter::convert(ADC::value(channel)).value;
    }
};

using voltageP = VoltageProvider<adcController, 0>;

template<typename ADC, uint8_t Channel>
struct CurrentProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
#ifdef FS_I6S
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE; // FS-I6S zeigt keinen Strom an
#else
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
#endif
//    using currentConverter = Hott::Units::Converter<adc, IBus::current_t, std::ratio<16450,1000>>; // todo: richtiger scale faktor (1k)
//    using currentConverter = Hott::Units::Converter<adc, IBus::current_t, std::ratio<10717,1000>>; // todo: richtiger scale faktor (1k5)
    using currentConverter = Hott::Units::Converter<adc, IBus::current_t, std::ratio<9189,1000>>; // todo: richtiger scale faktor (1k8)
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
#ifdef FS_I6S
        return currentConverter::convert(ADC::value(channel)).value / 10 + 400;
#else
        return currentConverter::convert(ADC::value(channel)).value;
#endif
    }
};

using currentP = CurrentProvider<adcController, 1>;

template<typename ADC, uint8_t Channel, typename UnInit>
struct TempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        if (UnInit::counter > 0) {
            return std::numeric_limits<uint16_t>::max() - UnInit::counter;
        }
        return sigrow::adcValueToTemperature<std::ratio<1,10>, 40 - 15>(ADC::value(channel)).value;
    }
};
using tempP = TempProvider<adcController, 2, uninitialzed>;

#ifdef USE_DAISY
struct IBusThrough {
    inline static void init() {
        daisyChain::template dir<Output>();
    }
    inline static void on() {
        daisyChain::on();
    }
    inline static void off() {
        daisyChain::off();
    }
};
using ibt = IBusThrough;
#else
using ibt = void;
#endif

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, Meta::List<voltageP, currentP, tempP>, systemTimer, ibt>;
#endif

#ifdef USE_HOTT
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;
using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t, std::ratio<121,21>>; // todo: richtiger scale faktor
using currentConverter = Hott::Units::Converter<adc, Hott::Units::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor
#endif

#if !(defined(USE_IBUS) || defined(USE_HOTT) || defined(USE_SPORT))
using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

#ifdef USE_SPORT
using portmux = Portmux::StaticMapper<Meta::List<tcaPosition>>;
#else
using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;
#endif

#ifdef USE_HOTT
class RCMenu final : public Hott::Menu<Parameter::menuLines> {
public:
    RCMenu() : Menu(this, "OnOff 1.0"_pgm, &mSoft, &mTimeout, &mType) {}
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
#endif

template<typename PWM, typename Output>
struct PwmWrapper {
    inline static constexpr auto f_timer = PWM::f_timer;
    inline static void init() {
        PWM::init();
    }
    inline static void off() {
        PWM::template off<Meta::List<Output>>();
    }
    inline static void on() {
        PWM::template on<Meta::List<Output>>();
    }
    inline static void frequency(uint16_t f) {
        PWM::frequency(f);
    }
    inline static void duty(uint16_t d) {
        PWM::template duty<Meta::List<Output>>(d);
    }
};

using music_speed = std::integral_constant<uint16_t, 100>;

using tonePwm = PWM::DynamicPwm<tcaPosition>;
using toneWrapper = PwmWrapper<tonePwm, AVR::PWM::WO<0>>;
using toneGenerator = External::Music::Generator<toneWrapper, music_speed>;

namespace {
    using namespace External::Music;
    
    using note = Note<toneWrapper, music_speed>;
    
    constexpr note c_ii_0 {Pitch{Letter::c}, Length{Base::doublenote}};
    
    constexpr note c_ii_1 {Pitch{Letter::c}, Length{Base::whole}};
    constexpr note c_s_ii_1 {Pitch{Letter::c, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note d_ii_1 {Pitch{Letter::d}, Length{Base::whole}};
    constexpr note d_s_ii_1 {Pitch{Letter::d, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note e_ii_1 {Pitch{Letter::e}, Length{Base::whole}};
    constexpr note f_ii_1 {Pitch{Letter::f}, Length{Base::whole}};
    constexpr note f_s_ii_1 {Pitch{Letter::f, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note g_ii_1 {Pitch{Letter::g}, Length{Base::whole}};
    constexpr note g_s_ii_1 {Pitch{Letter::g, Octave::ii, Accidential::sharp}, Length{Base::whole}};
    constexpr note a_ii_1 {Pitch{Letter::a}, Length{Base::whole}};
    constexpr note h_ii_1 {Pitch{Letter::h}, Length{Base::whole}};
    
    constexpr note c_ii_2 {Pitch{Letter::c}, Length{Base::half}};
    constexpr note e_ii_2 {Pitch{Letter::e}, Length{Base::half}};
    constexpr note g_ii_2 {Pitch{Letter::g}, Length{Base::half}};
    
    constexpr note c_ii_4 {Pitch{Letter::c}, Length{Base::quarter}};
    constexpr note c_s_ii_4 {Pitch{Letter::c, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note d_ii_4 {Pitch{Letter::d}, Length{Base::quarter}};
    constexpr note d_s_ii_4 {Pitch{Letter::d, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note e_ii_4 {Pitch{Letter::e}, Length{Base::quarter}};
    constexpr note f_ii_4 {Pitch{Letter::f}, Length{Base::quarter}};
    constexpr note f_s_ii_4 {Pitch{Letter::f, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note g_ii_4 {Pitch{Letter::g}, Length{Base::quarter}};
    constexpr note g_s_ii_4 {Pitch{Letter::g, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note a_ii_4 {Pitch{Letter::a}, Length{Base::quarter}};
    constexpr note h_ii_4 {Pitch{Letter::h}, Length{Base::quarter}};
    
    constexpr note c_iii_1 {Pitch{Letter::c, Octave::iii}, Length{Base::whole}};
    
    constexpr note c_iii_4 {Pitch{Letter::c, Octave::iii}, Length{Base::quarter}};
    constexpr note c_s_iii_4 {Pitch{Letter::c, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note d_iii_4 {Pitch{Letter::d, Octave::iii}, Length{Base::quarter}};
    constexpr note d_s_iii_4 {Pitch{Letter::d, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note e_iii_4 {Pitch{Letter::e, Octave::iii}, Length{Base::quarter}};
    constexpr note f_iii_4 {Pitch{Letter::f, Octave::iii}, Length{Base::quarter}};
    constexpr note f_s_iii_4 {Pitch{Letter::f, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note g_iii_4 {Pitch{Letter::g, Octave::iii}, Length{Base::quarter}};
    constexpr note g_s_iii_4 {Pitch{Letter::g, Octave::iii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note a_iii_4 {Pitch{Letter::a, Octave::iii}, Length{Base::quarter}};
    constexpr note a_iii_1 {Pitch{Letter::a, Octave::iii}, Length{Base::whole}};
    constexpr note h_iii_4 {Pitch{Letter::h, Octave::iii}, Length{Base::quarter}};
    
    constexpr note pause_4 {Pitch{Letter::pause}, Length{Base::quarter}};
    constexpr note pause_1 {Pitch{Letter::pause}, Length{Base::whole}};
    
    constexpr auto startMelody = AVR::Pgm::Array<note, c_ii_1, d_ii_1, e_ii_1, f_ii_1, g_ii_1, a_ii_1, h_ii_1, c_iii_1>{}; // C-Dur up
    constexpr auto sleepMelody = AVR::Pgm::Array<note, c_iii_1, g_ii_1, e_ii_1, c_ii_1>{}; // C-Dur 3-tone down
    
    constexpr auto waitOnMelody = AVR::Pgm::Array<note, c_ii_1, f_s_ii_1>{}; // Tritonus
    constexpr auto waitOffMelody = AVR::Pgm::Array<note, c_iii_1, h_ii_1, c_iii_1, a_ii_1, c_iii_1, g_ii_1, c_iii_1, f_ii_1, c_iii_1, e_ii_1, c_iii_1, d_ii_1, c_iii_1, c_ii_1>{};
    
    constexpr auto onMelody = AVR::Pgm::Array<note, pause_4, c_ii_4, pause_4, c_ii_4, pause_4, c_ii_2, e_ii_2, g_ii_2, c_iii_1>{};
    constexpr auto offMelody = AVR::Pgm::Array<note, c_ii_0, pause_1, c_ii_0, pause_1, c_ii_0>{};
    
}

template<typename Timer, typename Wdt, typename UnInit>
struct FSM {
    enum class State : uint8_t {Init, Startup, Idle, WaitSleep, Sleep, WaitOn, WaitOnInterrupted, On, FetOn, WaitOff, WaitOffInterrupted, WdtOn};
    
    static constexpr auto intervall = Timer::intervall;
    
    static constexpr External::Tick<Timer> idleTimeBeforeSleepTicks{10000_ms};
    
    //    std::integral_constant<uint16_t, idleTimeBeforeSleepTicks.value>::_;

    static constexpr External::Tick<Timer> softStartTicks{100_ms};
    
    inline static External::Tick<Timer> onDelay;
    
    inline static void init() {
        load();
        if (std::any(UnInit::value & 0x01_B)) {
            mState = State::WdtOn;
        }
    }    
    
    inline static void load() {
#ifdef USE_EEPROM
        if (appData[Storage::AVKey::TimeOut]) {
            onDelay = 1000_ms * appData[Storage::AVKey::TimeOut].toInt();
        }
#else
        onDelay = 1000_ms;
#endif
    }    
    
//    inline static uint16_t testCount = 0;
    
    inline static void periodic() {
        Wdt::reset();
//        if (testCount < 40000) {
//            Wdt::reset();
//            ++testCount;
//        }
        const State lastState = mState;
        ++stateTicks;
        switch (mState) {
        case State::Init:
            fet::inactivate();
            toneGenerator::play(startMelody);
            mState = State::Startup;
            break;
        case State::WdtOn:
            mState = State::FetOn;
            break;
        case State::Startup:
            if (!toneGenerator::busy()) {
                mState = State::Idle;
            }
            break;
        case State::Idle:
            if (stateTicks > idleTimeBeforeSleepTicks) {
                mState = State::WaitSleep;
            }
            else {
                if (button::event() == button::Press::Short) {
                    mState = State::WaitOn;
                }
            }
            break;
        case State::WaitSleep:
            if (!toneGenerator::busy()) {
                mState = State::Sleep;
            }
            break;
        case State::Sleep:
            if (button::event() == button::Press::Short) {
                mState = State::WaitOn;
            }
            break;
        case State::WaitOn:
            if (auto event = button::event(); event == button::Press::Long) {
                mState = State::On;
            }
            else if (event == button::Press::Release) {
                mState = State::WaitOnInterrupted;
            }
            break;
        case State::WaitOnInterrupted:
            mState = State::Idle;
            break;
        case State::On:
            if (stateTicks > onDelay) {
                mState = State::FetOn;
            }
            if (button::event() == button::Press::Short) {
                mState = State::WaitOff;
            }
            break;
        case State::FetOn:
            if (button::event() == button::Press::Short) {
                mState = State::WaitOff;
            }
            break;
        case State::WaitOff:
            if (auto event = button::event(); event == button::Press::Long) {
                mState = State::Idle;
            }
            else if (event == button::Press::Release) {
                mState = State::WaitOffInterrupted;
            }
            break;
        case State::WaitOffInterrupted:
            mState = State::On;
            break;
            //        default:
            //            break;
        }
        if (lastState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Init:
                break;
            case State::WdtOn:
                break;
            case State::Startup:
                fet::inactivate();
                break;
            case State::WaitSleep:
                led::off();
                toneGenerator::play(sleepMelody);
                break;
            case State::Sleep:
                fet::inactivate();
                UnInit::value = 0x00_B;
                led::off();
                wdt::off<ccp>();
                sleep::down();
                wdt::init<ccp>();
                mState = State::Idle;
                break;
            case State::WaitOn:
                toneGenerator::play(waitOnMelody, true);
                led::blink(led::count_type{2});
                break;
            case State::WaitOnInterrupted:
                toneGenerator::play(h_iii_4);
                break;                
            case State::WaitOffInterrupted:
                toneGenerator::play(h_iii_4);
                break;                
            case State::On:
                if (lastState != State::WaitOffInterrupted) {
                    toneGenerator::play(onMelody);
                }
                led::blink(led::count_type{4});
                break;
            case State::FetOn:
                if (!toneGenerator::busy()) {
                    if (lastState != State::WdtOn) {
                        toneGenerator::play(a_iii_1);
                    }
                }
                led::blink(led::count_type{5});
                fet::activate();
                UnInit::value = 0x01_B;
                break;
            case State::WaitOff:
                led::blink(led::count_type{3});
                toneGenerator::play(waitOffMelody, true);
                break;
            case State::Idle:
                if (lastState != State::WaitOnInterrupted) {
                    toneGenerator::play(offMelody);
                }
                led::blink(led::count_type{1});
                fet::inactivate();
                UnInit::value = 0x00_B;
                break;
            }
        }
    }
private:
    inline static External::Tick<Timer> stateTicks;
    inline static State mState{State::Init};    
};

using fsm = FSM<systemTimer, wdt, uninitialzed>;

#ifdef USE_HOTT
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
#endif

int main() {
    wdt::init<ccp>();

    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    reset::noWatchDog([]{
        uninitialzed::reset();
    });

    reset::onWatchDog([]{
        uninitialzed::counter = uninitialzed::counter + 1;        
    });
    
    
    fet::init();
#ifdef USE_EEPROM
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
#endif
    
    buttonPin::attributes<Meta::List<Attributes::Interrupt<Attributes::BothEdges>>>();
    button::init();
    
    led::init();
    portmux::init();
    systemTimer::init();
    adcController::init();
    
    toneGenerator::init();    

    fsm::init();
    
#ifndef NDEBUG
    dbg1::template dir<Output>();
#endif
    
#ifdef USE_HOTT
    sensor::init();
    menu::init();
#endif
#ifdef USE_IBUS
    ibus::init();
#endif
#ifdef USE_SPORT
    sensor::init();
#endif
#if !(defined(USE_HOTT) || defined(USE_IBUS) || defined(USE_SPORT))
    terminalDevice::init<AVR::BaudRate<9600>>();
#endif
    
    sleep::template init<sleep::PowerDown>();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    const auto capTimer = alarmTimer::create(200_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        buttonPin::resetInt();
        etl::Scoped<etl::EnableInterrupt<>> ei;
#if !(defined(USE_IBUS) || defined(USE_HOTT) || defined(USE_SPORT))
        etl::outl<terminal>("*"_pgm);
#endif
        while(true) {
#ifdef USE_HOTT
            sensor::periodic();
            menu::periodic();
#endif
#ifdef USE_IBUS
            ibus::periodic();
#endif
#ifdef USE_SPORT
            sensor::periodic();
#endif
#if !(defined(USE_IBUS) || defined(USE_HOTT) || defined(USE_SPORT))
            terminalDevice::periodic();
#endif
#ifdef USE_EEPROM
            bool eepSave{};
            eeprom::saveIfNeeded([&]{
                fsm::load();                
                eepSave = true;
            });
#endif
            adcController::periodic();
            
            systemTimer::periodic([&]{
#ifdef USE_HOTT
                sensor::ratePeriodic();
                measurements::periodic();
#endif
#ifdef USE_IBUS
                ibus::ratePeriodic();
#endif
#ifdef USE_SPORT
//            sensor::ratePeriodic();
#endif
                toneGenerator::periodic();
                fsm::periodic();
                
                button::periodic();
                led::periodic();
                
#ifndef NDEBUG
                dbg1::toggle();
#endif
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
#if !(defined(USE_IBUS) || defined(USE_HOTT) || defined(USE_SPORT))
                        if (eepSave) {
                            etl::outl<terminal>("es"_pgm);
                            eepSave = false;
                        }
                        if (changed) {
                            etl::outl<terminal>("ni"_pgm);
                        }
                        etl::outl<terminal>("test05 v: "_pgm, adcController::value(adcController::index_type{0}).toInt(), 
                                            " c: "_pgm, adcController::value(adcController::index_type{1}).toInt(),
                                            " t: "_pgm, adcController::value(adcController::index_type{2}).toInt(),
                                            " k: "_pgm, sigrow::adcValueToTemperature(adcController::value(adcController::index_type{2})).value
                                            );
#endif
                    }
                    if (capTimer == t) {
#ifdef USE_HOTT
                        measurements::tick();
#endif
                    }
                });
#ifdef USE_EEPROM
                appData.expire();
#endif
            });
        }
    }
}

ISR(PORTA_PORT_vect) {
    buttonPin::resetInt();
}

#ifdef USE_SPORT
ISR(PORTB_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<rxPin::name_type>>();
}

ISR(TCD0_OVF_vect) {
    isrRegistrar::isr<AVR::ISR::Tcd<0>::Ovf>();
}
#endif

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
