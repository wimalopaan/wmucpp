//#define USE_IBUS

#define USE_SPORT
//#define USE_HOTT

#include "board.h"
#include "temp.h"

//using dbg1 = so4Pin;

using adcController = External::Hal::AdcController<adc, Meta::NList<10, 11, 5, 3, 0x1e>>; // 1e = temp

template<typename ADC, uint8_t Channel>
struct RawProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::ARMED;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return ADC::value(channel).toInt();
    }
};

#ifdef USE_SPORT
using rxPin = Pin<Port<A>, 1>; // alt1
using txPin = rxPin;
template<typename PA>
using sensorUsart = External::SoftSerial::Usart<Meta::List<rxPin, txPin>, Component::Tcd<0>, 
PA, AVR::BaudRate<57600>,
AVR::ReceiveQueueLength<0>,
AVR::SendQueueLength<64>,
etl::NamedFlag<true>
>;

using a1 = External::AnalogSensor<adcController, 0, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;
using a2 = External::AnalogSensor<adcController, 1, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;
using a3 = External::AnalogSensor<adcController, 2, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;
using a4 = External::AnalogSensor<adcController, 3, std::ratio<500,1000>, std::ratio<1,100>, std::ratio<1,1>>;

template<typename Sensor>
struct TempProvider {
    inline static constexpr auto valueId = External::SPort::ValueId::Temp1;
    inline static uint32_t value() {
        return Sensor::value();
    }
};

using p1 = TempProvider<a1>;
using p2 = TempProvider<a2>;
using p3 = TempProvider<a3>;
using p4 = TempProvider<a4>;

using sport = External::SPort::Sensor<External::SPort::SensorId::ID3, sensorUsart, systemTimer, 
                                       Meta::List<p1, p2, p3, p4>>;

using isrRegistrar = IsrRegistrar<sport::uart::StartBitHandler, sport::uart::BitHandler>;

using portmux = Portmux::StaticMapper<Meta::List<>>;
#endif

#ifdef USE_IBUS
using raw0P = RawProvider<adcController, 0>;
using raw1P = RawProvider<adcController, 1>;
using raw2P = RawProvider<adcController, 2>;
using raw3P = RawProvider<adcController, 3>;

using iTempP = InternalTempProvider<adcController, 4>;

using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<raw0P, raw1P, raw2P, raw3P>, 
                          systemTimer, ibt>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;
#endif

#ifdef USE_HOTT
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;
using battVoltageConverter = Hott::Units::Converter<adc, Hott::Units::battery_voltage_t, std::ratio<121,21>>; // todo: richtiger scale faktor
using currentConverter = Hott::Units::Converter<adc, Hott::Units::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor
using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

auto sensorData = Hott::Experimental::Adapter<Hott::GamMsg>(sensor::data());

struct RCMenu final : public Hott::Menu<Parameter::menuLines> {
    RCMenu() : Menu(this, "TSensor 1.0"_pgm, &mTemp1, &mTemp2, &mTemp3, &mTemp4) {
        etl::itoa(42, t1);
        etl::itoa(43, t2);
        etl::itoa(44, t3);
        etl::itoa(45, t4);
        
        sensorData.temp1(etl::FixedPoint<int, 4>::fromRaw(42));
    }
    inline static constexpr uint8_t valueTextLength{6};
private:
    etl::StringBuffer<valueTextLength> t1;
    etl::StringBuffer<valueTextLength> t2;
    etl::StringBuffer<valueTextLength> t3;
    etl::StringBuffer<valueTextLength> t4;
    
    Hott::TextItem<valueTextLength> mTemp1{"T1"_pgm, t1};
    Hott::TextItem<valueTextLength> mTemp2{"T2"_pgm, t2};
    Hott::TextItem<valueTextLength> mTemp3{"T3"_pgm, t3};
    Hott::TextItem<valueTextLength> mTemp4{"T4"_pgm, t4};
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
    
    portmux::init();
    systemTimer::init();
    adcController::init();
    
#ifdef USE_IBUS
    ibus::init();
#endif
#ifdef USE_SPORT
    sport::init();
#endif
#ifdef USE_HOTT
    sensor::init();
#endif
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    {
#ifdef USE_SPORT
        etl::Scoped<etl::EnableInterrupt<>> ei;        
#endif
        while(true) {
#ifdef USE_IBUS
            ibus::periodic();
#endif
#ifdef USE_SPORT
            sport::periodic();
#endif
#ifdef USE_HOTT
            sensor::periodic();
            menu::periodic();
#endif
            adcController::periodic();
            
            systemTimer::periodic([&]{
                wdt::reset();
#ifdef USE_IBUS
                ibus::ratePeriodic();
#endif
#ifdef USE_HOTT
                sensor::ratePeriodic();
#endif
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                    }
                });
            });
        }
    }
}

#ifdef USE_SPORT
ISR(PORTA_PORT_vect) {
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
