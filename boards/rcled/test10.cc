#define NDEBUG

//#define USE_IBUS

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
#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/solutions/rc/busscan.h>

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

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

struct Data final : public EEProm::DataBase<Data> {
    uint8_t mMagic{};
    
    using search_t = etl::uint_ranged_circular<uint8_t, 0, 15>;
    search_t mChannel{};
    
    uint16_t mThrMin{0};
    uint16_t mThrMax{65535};
    uint16_t mThrMid{32000};
    //    uint16_t mThrHalf{16000};
    uint16_t mThrDead{16000};
    
};

template<template<typename Command> typename Spi, AVR::Concepts::Pin OePin, AVR::Concepts::Pin IRef1Pin, AVR::Concepts::Pin IRef2Pin>
struct PCA9745 {
    struct Ramp {
        std::byte rate{};
        std::byte step{};
        std::byte hold{};
        std::byte iref{};
    };
    struct Registers {
        std::byte mode0{};
        std::byte mode1{};
        std::array<std::byte, 4> ledouts{};
        std::byte grppwm{};
        std::byte grpfreq{};
        std::array<std::byte, 16> pwms{};
        std::array<std::byte, 16> irefs{};
        std::array<Ramp, 4> ramps{};
        std::array<std::byte, 2> grad_modes{};
        std::array<std::byte, 4> grad_grps{};
        std::byte grad_cntl;
        std::byte offset;
        std::byte pwmall;
        std::byte irefall;
    };
    static_assert(sizeof(Registers) == 0x42);
    
    
    struct Address {
        std::byte raw{0xff}; 
    };
    
    struct Read{};
    struct Write{};
    
    struct Command {
        using value_type = std::byte;
        
        Command() = default;
        
        explicit Command(Write, uint8_t a, std::byte v) : address{((std::byte{a} & 0x7f_B) << 1) | 0x00_B}, data{v} {}        
        
        template<uint8_t N>
        std::byte get() const {
            if constexpr(N == 0) {
                return address.raw;
            }
            else if constexpr(N == 1) {
                return data;
            }
            else {
                static_assert(std::false_v<Spi>);
            }
        }
        constexpr const std::byte& operator[](uint8_t i) const {
            if (i == 0) {
                return address.raw;
            }         
            return data;
        }
        static constexpr uint8_t size() {
            return 2;
        }
    private:
        Address address{};
        std::byte data{0xff};
    };
    
    using spi = Spi<Command>;
    using oePin = OePin;
    
    using iref1Pin = IRef1Pin;
    using iref2Pin = IRef2Pin;
    
    enum class Current : uint8_t {C0, C1, C2, C3};
    
    static inline void current(const Current c) {
        switch(c) {
        case Current::C0:
            iref1Pin::template dir<AVR::Input>();
            iref2Pin::template dir<AVR::Input>();
            break;
        case Current::C1:
            iref1Pin::template dir<AVR::Input>();
            iref2Pin::template dir<AVR::Output>();
            iref2Pin::low();
            break;
        case Current::C2:
            iref1Pin::template dir<AVR::Output>();
            iref1Pin::low();
            iref2Pin::template dir<AVR::Input>();
            break;
        case Current::C3:
            iref1Pin::template dir<AVR::Output>();
            iref1Pin::low();
            iref2Pin::template dir<AVR::Output>();
            iref2Pin::low();
            break;
        }
    }    
    
    static inline void init() {
        current(Current::C1);
        
        oePin::template dir<AVR::Output>();
        oePin::low();
        
        spi::init();
        spi::put(Command{Write{}, 0, 0x00_B}); // normal mode
        spi::put(Command{Write{}, 1, 0x14_B}); // clear / exponnetial
        //        spi::put(Command{Write{}, 0x40, 130_B});
        spi::put(Command{Write{}, 0x41, 10_B}); // pwmall
        
        // group 0
        spi::put(Command{Write{}, 0x28, 0xc1_B}); // ramp up/down
        spi::put(Command{Write{}, 0x29, 0x04_B}); // step time
        spi::put(Command{Write{}, 0x2a, 0xd2_B}); // hold 
        spi::put(Command{Write{}, 0x2b, 200_B}); // iref
        
        // group 1
        spi::put(Command{Write{}, 0x2c, 0xc1_B}); // ramp up/down
        spi::put(Command{Write{}, 0x2d, 0x02_B}); // step time
        spi::put(Command{Write{}, 0x2e, 0xc9_B}); // hold 
        spi::put(Command{Write{}, 0x2f, 100_B}); // iref
         
        spi::put(Command{Write{}, 0x38, 0xff_B}); // gradation channel 0-7
        spi::put(Command{Write{}, 0x3a, 0b0000'0100_B}); // groups: led0=group0, led1=group1
        
        spi::put(Command{Write{}, 0x3e, 0x0f_B}); // start 0, 1 continous
        
        spi::put(Command{Write{}, 2, 0b0000'0101_B}); // led 0,1 fully on
        
    }
    
    template<bool on>
    static inline void out(const uint8_t i) {
        //        if constexpr(on) {
        //            spi::put(Command{Write{}, 2, 0x01_B});            
        //        }
        //        else {
        //            spi::put(Command{Write{}, 2, 0x00_B});                        
        //        }
    }
    static inline void periodic() {
        spi::periodic();
    }
private:
    static inline Registers registers;
    
};

template<typename BusDevs>
struct GlobalFSM {
    using bus_type = BusDevs::bus_type;
    using devs = bus_type::devs;
    using pca = devs::pca9745;
    using led = devs::scanLedPin;
    
    using timer = BusDevs::systemTimer;
    
    using servo = BusDevs::servo;    
    using servo_pa = BusDevs::servo_pa;
    
    using term = BusDevs::terminal;
    using termDev = term::device_type;
    
    using nvm = BusDevs::eeprom;
    
    inline static void init(const bool invert) {
        led::inactivate();
        nvm::init();
        pca::init();
        //        if (data.mMagic != BusDevs::magic) {
        //            data.mMagic = BusDevs::magic;
        //            nvmDefaults<true>();
        //            etl::outl<terminal>("e init"_pgm);
        //        }
        
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            servo::template init<BaudRate<115200>>();
            etl::outl<term>("IB"_pgm);
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (invert) {
                servo::rxInvert(true);
                etl::outl<term>("SB I"_pgm);
            }
            else {
                etl::outl<term>("SB"_pgm);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
    }
    inline static void periodic() {
        pca::periodic();        
        servo::periodic();
    }
    inline static void ratePeriodic() {
        servo_pa::ratePeriodic();
        
    }
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using iref1Pin = Pin<Port<A>, 6>;
    using iref2Pin = Pin<Port<A>, 7>;
    
    using csPin = Pin<Port<B>, 0>;
    using oePin = Pin<Port<B>, 1>;
    
    using ledPin = Pin<Port<A>, 4>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
    
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; 
    using servoPosition = usart0Position;
    
    using scanDevPosition = usart0Position;
    using ppmDevPosition = void;
    using evrouter = void;
    //    using scan_term_dev = Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<32>>;
    using scan_term_dev = void;
    using scanLedPin = AVR::ActiveHigh<ledPin, Output>;
    
    using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
    
    template<typename Command>
    using spi = AVR::SpiSS<spiPosition, Command, AVR::QueueLength<16>, csPin>;
    
    using portmux = Portmux::StaticMapper<Meta::List<usart0Position, spiPosition>>;
    
    using pca9745 = PCA9745<spi, oePin, iref1Pin, iref2Pin>;
    
    static inline void init() {
        using portmux = Portmux::StaticMapper<Meta::List<usart0Position, spiPosition>>; 
        portmux::init();
        ccp::unlock([]{
            clock::template prescale<1>();
        });
        systemTimer::init(); 
    }
    static inline void periodic() {}
};

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    using bus_type = External::Bus::IBusIBus<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::High>;
        inline static constexpr auto stateProviderId = IBus2::Type::type::FLIGHT_MODE;
    };
    
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using terminal = etl::basic_ostream<servo>;
    
    using eeprom = EEProm::Controller<Data>;
    //    using eeprom = EEProm::Controller<Storage::ApplDataBus<typename servo_pa::channel_t, RCSwitch::addr_t, bus_type>>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::High>;
        inline static constexpr auto stateProviderId = IBus2::Type::type::FLIGHT_MODE;
    };
    
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using terminal = etl::basic_ostream<servo>;
    
    using eeprom = EEProm::Controller<Data>;
};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GlobalFSM<devs>;
            
            gfsm::init(inverted);
            etl::outl<terminal>("test10"_pgm);
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
using scanner = External::Scanner<devices, Application>;

int main() {
    scanner::run();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    //    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
        //        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
