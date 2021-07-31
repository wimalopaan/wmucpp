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
    
//    using pwm = devs::pwm;
    using driver = devs::driver;
    
    using encoder = devs::encoder;
    
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

        encoder::init();
        driver::init();
        
        dbg::template dir<Output>();
    }
    
    static inline void periodic() {
        Sensor::periodic();
        Servo::periodic();
        Adc::periodic();
        TermDev::periodic();
        
        encoder::periodic();
        driver::periodic();
    }

    static inline void ratePeriodic() {
        dbg::toggle();
        
        driver::ratePeriodic();
        
        servo_pa::ratePeriodic();
        blinkLed::ratePeriodic();
        Sensor::ratePeriodic();
    
        const auto oldState = mState;
        ++mStateTick;

        uint16_t ae{};
        
        encoder::startRead();
        const auto angle = encoder::readLast();
        if (angle) {
            ae = *angle;
        }
        
        (++mDebugTick).on(debugTicks, [&]{
            etl::outl<terminal>("*: "_pgm, ae, " tx: "_pgm, encoder::spi::txCount(), " rx: "_pgm, encoder::spi::rxCount());
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
                driver::angle(ae >> 2);
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S: Init"_pgm);
                blinkLed::blink(count_type{1});
                encoder::pwmOn(true);
                break;
            case State::Enable:
                etl::outl<terminal>("S: Enable"_pgm);
                enable::activate();
//                encoder::pwmOn(true);
                break;
            }
        }
    }
private:
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mDebugTick;
};

template<typename PWM>
struct Driver {
    using pwm = PWM;
    
    static inline constexpr uint16_t steps = 4096; // generate 8k-table (use O(log(N)) in pgm generator
    using index_type = etl::uint_ranged_circular<uint16_t, 0, steps - 1>;
    
    static inline constexpr uint16_t period = 2000;
    
    static inline constexpr uint8_t scale = 4;

    static inline constexpr uint8_t poles = 7;
    
    
    template<size_t Steps, size_t Shift>    
    struct Generator {
        constexpr auto operator()() {
            std::array<uint16_t, Steps> data;
            for(uint16_t i = 0; i < Steps; ++i) {
                const uint16_t ie = i * poles;
                data[i] = (double)(period - 1) * (1.0 + sin(((ie + Shift) * 2.0 * M_PI) / Steps)) / 2.0;
            }
            return data;
        }
    };
    using Sine0 = AVR::Pgm::Util::Converter<Generator<steps, 0>>::pgm_type;
    using Sine1 = AVR::Pgm::Util::Converter<Generator<steps, (steps / 3)>>::pgm_type;
    using Sine2 = AVR::Pgm::Util::Converter<Generator<steps, 2 * (steps / 3)>>::pgm_type;
    
    static inline void init() {
        pwm::init();
        pwm::period(period);
        pwm::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
        setDuty();
    }
    static inline void angle(const uint16_t a) {
        index = index_type{a};
    }
private:    
    inline static void setDuty() {
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((scale * Sine0::value(index)) / 10);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((scale * Sine1::value(index)) / 10);
        PWM::template duty<Meta::List<AVR::PWM::WO<2>>>((scale * Sine2::value(index)) / 10);
    }

    inline static etl::uint_ranged_circular<uint16_t, 0, steps - 1> index{0};
};

namespace Util {
    namespace detail {
        struct Generatorx {
            constexpr auto operator()() {
                std::array<bool, 256> lut;
                for(uint8_t v{0}; auto& l : lut) {
                    l = etl::numberOfOnes(v) & 0x01;
                    ++v;
                }
                return lut;
            }
        };
        using EvenParity = AVR::Pgm::Util::Converter<Generatorx>::pgm_type;
    }    
    struct ParityGenerator {
        static constexpr inline bool isEvenParity(const std::byte v) {
            return detail::EvenParity::value(uint8_t(v));
        }
        static constexpr inline bool isEvenParity(const std::array<std::byte, 2>& v) {
            const std::byte x{v[0] ^ v[1]};
            return detail::EvenParity::value(uint8_t(x));
        }
//        static_assert(isEvenParity(0_B));
    };
}


namespace External::AS5147 {
    enum class Register : uint16_t {NOP = 0x000, ERRFL = 0x0001, PROG = 0x0003, DIAAGC = 0x3ffc, MAG = 0x3ffd, ANGLUNC = 0x3ffe, ANGLECOM = 0x3fff,
                                   SETTING1 = 0x0018, SETTING2 = 0x0019};

        
    struct Command {
        using value_type = std::byte;
        
        inline Command(const Register r = Register::NOP, const bool read = false) {
            mData[1] = etl::nth_byte<1>(uint16_t(r));
            mData[0] = etl::nth_byte<0>(uint16_t(r));
            if (read) {
                mData[1] |= 0x40_B;
            }
            bool p = ::Util::ParityGenerator::isEvenParity(mData);
            if (p) {
                mData[1] |= 0x80_B;
            }
        }
        inline Command(const std::byte d) {
            mData[1] = 0x00_B;
            mData[0] = d;
            bool p = ::Util::ParityGenerator::isEvenParity(d);
            if (p) {
                mData[1] |= 0x80_B;
            }
        }        
        inline constexpr const std::byte& operator[](const uint8_t i) const {
            if (i == 0) {
                return mData[1];
            }         
            return mData[0];
        }
        static constexpr uint8_t size() {
            return 2;
        }
    private:
        std::array<std::byte, 2> mData;
    };
    struct Result {
        using value_type = std::byte;
        constexpr std::byte& operator[](const uint8_t i) {
            if (i == 0) {
                return mData[1];
            }         
            return mData[0];
        }
        static constexpr uint8_t size() {
            return 2;
        }
    private:
        std::array<std::byte, 2> mData;
    };
    
    template<template<typename, typename> typename Spi>
    struct Encoder {
        using spi = Spi<Command, Result>;
        
        static inline void pwmOn(const bool b) {
            if (b) {
                spi::put(Command{Register::SETTING1});
                spi::put(Command{0x01_B << 7}); // pwm
            }
        }
        
        static inline void startRead() {
            spi::put(Command{Register::ANGLECOM, true});
            spi::put(Command{Register::ANGLECOM, true});
        }
        
        static inline std::optional<uint16_t> readLast() {
            Result r;
            if (spi::rxQueue().pop_front(r)) {
                const std::byte l = spi::rxQueue().front()[0];
                const std::byte h = (spi::rxQueue().front()[1] & 0x3f_B);
                spi::rxQueue().pop_front(r);
                const uint16_t v = uint16_t(h) << 8 | uint8_t(l);
                return {v};
            }
            return {};
        }

        static inline bool pending() {
            return spi::rxCount() >= 2;
            
        }
        
//        static inline uint8_t rxSize() {
//            return spi::rxQueue().size() ;
//        } 
        
        static inline void init() {
            spi::init();
            spi::mode1();
        }
        
        static inline void periodic() {
            spi::periodic();
        }
    private:
    };
}

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
    
    using ssPin = Pin<Port<A>, 7>; 
    
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

    using driver = Driver<pwm>;

    template<typename Command, typename Result>
    using spi = AVR::SpiBiSS<spiPosition, Command, Result, AVR::QueueLength<32>, ssPin>;
    
    using encoder = External::AS5147::Encoder<spi>;
    
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

            etl::outl<terminal>("foc_t02_hw01"_pgm);
            
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
