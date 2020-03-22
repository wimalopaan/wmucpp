#define NDEBUG

#define USE_IBUS

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
#include <mcu/pgm/pgmarray.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>
#include <external/ibus/ibus.h>
#include <external/units/music.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/blinker.h>

#include <external/solutions/debug/logic_analyzer.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 2048_Hz;
}

using systemTimer = SystemTimer<Component::Pit<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using ledPin    = ActiveHigh<Pin<Port<F>, 5>, Output>;
using assertPin = ActiveHigh<Pin<Port<F>, 6>, Output>;

using led0    = ActiveHigh<Pin<Port<D>, 0>, Output>;
using led1    = ActiveHigh<Pin<Port<D>, 1>, Output>;
using led2    = ActiveHigh<Pin<Port<D>, 2>, Output>;
using led3    = ActiveHigh<Pin<Port<D>, 3>, Output>;
using led4    = ActiveHigh<Pin<Port<D>, 4>, Output>;


using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; // I-Bus Sensor
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // I-Bus Servo
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>; // Terminal

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using sleep = Sleep<>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<8, 9, 0x1e>>; // 1e = temp

template<typename ADC, uint8_t Channel>
struct VoltageProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
//    index_t::_;
    inline static constexpr auto ibus_type = IBus::Type::type::EXTERNAL_VOLTAGE;
    inline static constexpr void init() {}
    
    using battVoltageConverter = Hott::Units::Converter<adc, IBus::battery_voltage_t, std::ratio<121,21>>; 
    
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
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
    using currentConverter = Hott::Units::Converter<adc, IBus::current_t, std::ratio<11117,1000>>; // todo: richtiger scale faktor
    inline static constexpr void init() {}
    inline static constexpr uint16_t value() {
        return currentConverter::convert(ADC::value(channel)).value;
    }
};

using currentP = CurrentProvider<adcController, 1>;

template<typename ADC, uint8_t Channel>
struct TempProvider {
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE;
    inline static constexpr void init() {
    }
    inline static constexpr uint16_t value() {
        return sigrow::adcValueToTemperature<std::ratio<1,10>, 40>(ADC::value(channel)).value;
    }
};

using tempP = TempProvider<adcController, 2>;

struct IBusThrough {
    inline static constexpr void on() {}
    inline static constexpr void off() {}
};
using ibt = IBusThrough;

using dbg = PinGroup<Meta::List<Pin<Port<A>, 3>, Pin<Port<A>, 4>, Pin<Port<A>, 5>, Pin<Port<A>, 6>, Pin<Port<A>, 7>> >;
using tParallelDebug = External::Debug::TriggeredParallel<Pin<Port<A>, 2>, dbg>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;

//using switch_provider = IBus::Switch::Provider<servo_pa>;
//using switch_provider2 = IBus::Switch::ProviderState<servo_pa>;

//using ibus = IBus::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<115200>, 
//                          Meta::List<voltageP, currentP, tempP>, systemTimer,
//                          ibt, tParallelDebug>;

using terminalDevice = AVR::Usart<usart2Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using servo = AVR::Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using ibus_switch = IBus::Switch::Switch<servo_pa>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position, tcaPosition>>;

int main() {
    assertPin::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    led0::init();
    led1::init();
    led2::init();
    led3::init();
    led4::init();
    
    ledPin::init();
    portmux::init();
    systemTimer::init();
    adcController::init();
    
//    ibus::init();
    
    terminalDevice::init<AVR::BaudRate<9600>>();
 
    servo::init<AVR::BaudRate<115200>>();
    
//    sleep::template init<sleep::PowerDown>();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        etl::outl<terminal>("test40"_pgm);
        
        while(true) {
            servo::periodic();
//            ibus::periodic();
            ibus_switch::periodic();
            
            terminalDevice::periodic();
            adcController::periodic();
            
            systemTimer::periodic([&]{
//                ibus::ratePeriodic();

                if (ibus_switch::states2w[0]) {                
                    led0::activate();            
                }
                else {
                    led0::inactivate();            
                }
                if (ibus_switch::states2w[1]) {                
                    led1::activate();            
                }
                else {
                    led1::inactivate();            
                }
                if (ibus_switch::states2w[2]) {                
                    led4::activate();            
                }
                else {
                    led4::inactivate();            
                }
                
                if (ibus_switch::states3w[0] == 1) {                
                    led2::activate();            
                }
                else {
                    led2::inactivate();            
                }
                if (ibus_switch::states3w[0] == 2) {                
                    led3::activate();            
                }
                else {
                    led3::inactivate();            
                }
                
                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        ledPin::toggle();
                        servo_pa::channel_t i0{0};
                        servo_pa::channel_t i4{4};
                        servo_pa::channel_t i5{5};
                        servo_pa::channel_t i6{6};
                        
                        etl::outl<terminal>(
//                                    " ch0: "_pgm, servo_pa::value(i0).toInt(),
//                                    " ch4: "_pgm, servo_pa::value(i4).toInt(),
                                    " ch5: "_pgm, servo_pa::value(i5).toInt()
//                                    " ch6: "_pgm, servo_pa::value(i6).toInt()
////                                    " s1: "_pgm, sv1,
////                                    " s2: "_pgm, sv2,
////                                    " s3: "_pgm, sv3,
////                                    " s4: "_pgm, sv4
////                                    " V: "_pgm, voltageP::value(),
////                                    " C: "_pgm, currentP::value(),
////                                    " T: "_pgm, tempP::value(),
////                                    " t: "_pgm, sigrow::adcValueToTemperature<std::ratio<1,10>, 40>(adcController::value(adcController::index_type(2))).value                
                                            ); 
                    }
                });
            });
        }
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
