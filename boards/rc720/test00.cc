#define NDEBUG

#define LEARN_DOWN

#ifndef GITMAJOR
# define VERSION_NUMBER 0001
#endif

#ifndef NDEBUG
static unsigned int assertKey{1234};
#endif

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>

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

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

template<typename Pin>
struct IBusThrough {
    inline static void init() {
        Pin::template dir<Output>();
    }
    inline static void on() {
        Pin::on();
    }
    inline static void off() {
        Pin::off();
    }
};

template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    
    using bus_type = BusDevs::bus_type;
    using devs = BusDevs::devs;
    using Timer = devs::systemTimer;
    
    using lut1 = devs::lut1;
    using Adc = devs::adcController;
    
    using Servo = BusDevs::servo;
    using servo_pa = BusDevs::servo_pa;
    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    using value_t = servo_pa::value_type;
    static inline constexpr value_t chThreshHPos = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshLPos = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};    
    static inline constexpr value_t chThreshHNeg = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshLNeg = value_t{value_t::Mid - (value_t::Upper - value_t::Lower) / 6};    
    
//    std::integral_constant<uint16_t, chThreshHPos.toInt()>::_;
    
    using Sensor = BusDevs::sensor;
    using sensor_pa = Sensor::ProtocollAdapter;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    using adc_i_t = Adc::index_type;
  
    enum class State : uint8_t {Undefined, Init};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    
    static inline void init(const bool) {
        
    }
    static inline void periodic() {
        
    }
    static inline void ratePeriodic() {
        
    }
    
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;

    using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>; // Servo / Debug
        
    using servoPosition = usart0Position; 
    using scanDevPosition = servoPosition;
    
    using sensorPosition = usart2Position; // Sensor
    using scanTermPosition = servoPosition;
#ifdef NDEBUG
    using scan_term_dev = void;
#else
    using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#endif
    
#ifndef NDEBUG
//    using assertPin = Pin<Port<A>, 0>; 
#endif
    
    using daisyChain= Pin<Port<D>, 4>; 
    
    using led1 = Pin<Port<F>, 1>;
    using led2 = Pin<Port<C>, 2>;
    
    using scanLedPin = ActiveHigh<led1, Output>;
    
    // lut1 out: pc3
    using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;
    using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Usart<1>, Ccl::Input::Mask>;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
    
    using ibt = IBusThrough<daisyChain>;
    
    using adc = Adc<Component::Adc<0>, AVR::Resolution<12>, Vref::V2_048>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<0, 19, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;
    
    using pwm3Position = Portmux::Position<Component::Tcb<0>, Portmux::Alt1>;
    using pwm4Position = Portmux::Position<Component::Tcb<1>, Portmux::Alt1>;
    using pwm5Position = Portmux::Position<Component::Tcb<2>, Portmux::Default>;

    
    using ppmDevPosition = void;

//    using evrouter = Event::Router<Event::Channels<>, Event::Routes<>>;
    using evrouter = void;
    
//    using eeprom = EEProm::Controller<Data>;
    
    using portmux = Portmux::StaticMapper<Meta::List<ccl1Position, tcaPosition, servoPosition, sensorPosition>>;
    
    static inline void init() {
        portmux::init();
        if constexpr(!std::is_same_v<evrouter, void>) {
//            evrouter::init();
        }
        ccp::unlock([]{
            if constexpr(AVR::Concepts::AtDa32<MCU>) {
                clock::template init<Project::Config::fMcuMhz>();
            }
            else if constexpr(AVR::Concepts::At01Series<MCU>) {
                clock::template prescale<1>();
            }
            else {
                static_assert(std::false_v<MCU>);
            }
        });
        systemTimer::init(); 
    }
    static inline void periodic() {
    }
};

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    static inline uint8_t magic = 42;
    using bus_type = External::Bus::IBusIBus<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename ADC, uint8_t Channel, typename SigRow>
    struct InternalTempProvider {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto ibus_type = IBus2::Type::type::TEMPERATURE;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,10>, 40, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };

    using adcController = devs::adcController;
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 6, typename Devs::sigrow>;
    
    using sensor = IBus2::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<115200>, 
                                Meta::List<tempiP>, 
                                systemTimer, typename devs::ibt>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    static inline uint8_t magic = 43;
    using bus_type = External::Bus::SBusSPort<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    template<typename PA>
    using sensorUsart = AVR::Usart<typename Devs::sensorPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<64>>;
    
    template<typename ADC, uint8_t Channel, typename SigRow>
    struct InternalTempProvider {
        using index_t = ADC::index_type;
        static_assert(Channel <= index_t::Upper);
        inline static constexpr auto channel = index_t{Channel};
        inline static constexpr auto valueId = External::SPort::ValueId::Temp2;
        inline static constexpr void init() {}
        inline static constexpr uint32_t value() {
            return SigRow::template adcValueToTemperature<std::ratio<1,1>, 0, typename ADC::VRef_type>(ADC::value(channel)).value;
        }
    };


    using adcController = devs::adcController;
    
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 6, typename Devs::sigrow>;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, 
                                           Meta::List<tempiP>>;

};

template<typename Devs>
struct BusDevs<External::Bus::SumDHott<Devs>> {
    static inline uint8_t magic = 44;
    using bus_type = External::Bus::SumDHott<Devs>;
    using devs = Devs;
    
    struct BusParam {
        using bus_t = bus_type;
    };
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
       
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif

    using configProvider = void;
    
    using sensor = Hott::Experimental::Sensor<typename Devs::sensorPosition, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;

    using calibratePs = Meta::List<>;

};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isSBus<BusSystem>::value || External::Bus::isIBus<BusSystem>::value || External::Bus::isSumD<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GlobalFsm<devs>;
            
            gfsm::init(inverted);

            etl::outl<terminal>("esc10_avr128_hw03"_pgm);
            
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
using scanner = External::Scanner2<devices, Application, Meta::List<External::Bus::IBusIBus<devices>, External::Bus::SBusSPort<devices>>>;

int main() {
    scanner::run();
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    xassert::ab.clear();
    xassert::ab.insertAt(0, expr);
    etl::itoa(line, xassert::aline);
    xassert::ab.insertAt(20, xassert::aline);
    xassert::ab.insertAt(30, file);
    xassert::on = true;
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
