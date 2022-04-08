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

    using blinkLed = External::Blinker2<typename devs::led, Timer, 100_ms, 2500_ms>;
    using count_type = blinkLed::count_type;
    
    using enable = devs::enable;

    using dbg = devs::dbgPin;
    
    using pwm = devs::pwm;
    
    using lut3 = devs::lut3;
    using Adc = devs::adcController;
    
    using Servo = BusDevs::servo;
    using servo_pa = BusDevs::servo_pa;
    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    using value_t = servo_pa::value_type;
    
    using Sensor = BusDevs::sensor;
    using sensor_pa = Sensor::ProtocollAdapter;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    using adc_i_t = Adc::index_type;
  
    enum class State : uint8_t {Undefined, Init, Enable};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> enableTicks{1000_ms};
    static constexpr External::Tick<Timer> debugTicks{1000_ms};
    
    static inline void init(const bool inverted) {
        blinkLed::init();
        enable::init();
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            lut3::init(std::byte{0x00}); // low on lut3-out 
            Sensor::init();
            Sensor::uart::txOpenDrain();
            Servo::template init<BaudRate<115200>>();

            etl::outl<terminal>("IB"_pgm);
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            lut3::init(std::byte{0x33}); // route TXD (inverted) to lut1-out 
            Sensor::init();
            Sensor::uart::txPinDisable();

            devs::ibt::init();
            devs::ibt::off();
            
            Servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (inverted) {
                Servo::rxInvert(true);
                etl::outl<terminal>("SB I"_pgm);
            }
            else {
                etl::outl<terminal>("SB"_pgm);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
        
        Adc::template init<false>(); // no pullups
        Adc::mcu_adc_type::nsamples(4);

        pwm::init();
        
        dbg::template dir<Output>();
    }
    
    static inline void periodic() {
        Sensor::periodic();
        Servo::periodic();
        Adc::periodic();
        TermDev::periodic();
    }

    static inline void ratePeriodic() {
        dbg::toggle();
        
        servo_pa::ratePeriodic();
        blinkLed::ratePeriodic();
        Sensor::ratePeriodic();
    
        const auto oldState = mState;
        ++mStateTick;
        
        (++mDebugTick).on(debugTicks, []{
            etl::outl<terminal>("*"_pgm);
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(enableTicks, []{
                mState = State::Enable;
            });
            break;
        case State::Enable:
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S: Init"_pgm);
                blinkLed::blink(count_type{1});
                break;
            case State::Enable:
                etl::outl<terminal>("S: Enable"_pgm);
                enable::activate();
                pwm::period(2000);
                pwm::template duty<Meta::List<AVR::PWM::WO<2>>>(100);
                pwm::template on<Meta::List<AVR::PWM::WO<2>>>();
                break;
            }
        }
    }
private:
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;

    using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
    
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; // terminal
    using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo (in / out)
    using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>; // Sensor

    using servoPosition = usart1Position; 
    using scanDevPosition = servoPosition;
    
    using sensorPosition = usart2Position; // Sensor
    using scanTermPosition = usart0Position;

    using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
    
    using dbgPin = Pin<Port<A>, 3>; 
    
    using daisyChain= Pin<Port<F>, 5>; 

    using pwmInPin = Pin<Port<C>, 3>;

    using WPin = Pin<Port<D>, 0>;
    using IPin = WPin;
    using VPin = Pin<Port<D>, 1>;
    using BPin = VPin;
    using UPin = Pin<Port<D>, 2>;
    using APin = UPin;

    using ledPin = Pin<Port<D>, 6>;
    using led = ActiveHigh<ledPin, Output>;
    
    using enablePin = Pin<Port<D>, 7>;
    using enable = ActiveHigh<enablePin, Output>;
    
    using scanLedPin = ActiveHigh<ledPin, Output>;
    
    // lut3 out: pf3
    using ccl3Position = Portmux::Position<Component::Ccl<3>, Portmux::Default>;
    using lut3 = Ccl::SimpleLut<3, Ccl::Input::Mask, Ccl::Input::Mask,Ccl::Input::Usart<2>>;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltF>;
    using pwm = External::PWM::DynamicPwm<tcaPosition>;
    
    using ibt = IBusThrough<daisyChain>;
    
    using adc = Adc<Component::Adc<0>, AVR::Resolution<12>, Vref::V2_048>; // 3 = Strom, 4 = VBatt, 5 = Temp
    using adcController = External::Hal::AdcController<adc, Meta::NList<3, 4, 5, 0x42>>; // 0x42 = temp
    using adc_i_t = adcController::index_type;
    
    using ppmDevPosition = void;

//    using evrouter = Event::Router<Event::Channels<>, Event::Routes<>>;
    using evrouter = void;
    
//    using eeprom = EEProm::Controller<Data>;
    
    using portmux = Portmux::StaticMapper<Meta::List<ccl3Position, tcaPosition, servoPosition, sensorPosition, scanTermPosition>>;
    
    static inline void init() {
        portmux::init();
        if constexpr(!std::is_same_v<evrouter, void>) {
//            evrouter::init();
        }
        ccp::unlock([]{
            clock::template init<Project::Config::fMcuMhz>();
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

    using terminal = etl::basic_ostream<typename devs::scan_term_dev>;
    
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
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 3, typename Devs::sigrow>;
    
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
    
    using terminal = etl::basic_ostream<typename devs::scan_term_dev>;
    
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
    
    using tempiP = InternalTempProvider<typename Devs::adcController, 3, typename Devs::sigrow>;
    
    using sensor = External::SPort::Sensor<External::SPort::SensorId::ID1, sensorUsart, systemTimer, 
                                           Meta::List<tempiP>>;

};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using busdevs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isSBus<BusSystem>::value || External::Bus::isIBus<BusSystem>::value) {
            using terminal = busdevs::terminal;
            using systemTimer = busdevs::systemTimer;
            using gfsm = GlobalFsm<busdevs>;
            
            gfsm::init(inverted);

            etl::outl<terminal>("foc01_hw01"_pgm);
            
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
