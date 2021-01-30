//#define NDEBUG

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
#include <external/ibus/ibus.h>

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
        
        Command() {}
        
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
    
    static inline void init() {
        iref1Pin::template dir<AVR::Output>();
        iref2Pin::template dir<AVR::Input>();
        oePin::template dir<AVR::Output>();
        iref1Pin::low();
//        iref2Pin::high();
        oePin::low();
        spi::init();
        spi::put(Command{Write{}, 0, 0x00_B});
        spi::put(Command{Write{}, 1, 0x14_B});
//        spi::put(Command{Write{}, 0x40, 130_B});
        spi::put(Command{Write{}, 0x41, 10_B});

        spi::put(Command{Write{}, 0x28, 0xc1_B}); // ramp up/down
        spi::put(Command{Write{}, 0x29, 0x04_B}); // step time
        spi::put(Command{Write{}, 0x2a, 0xd2_B}); // hold 
        spi::put(Command{Write{}, 0x2b, 200_B}); // iref

        spi::put(Command{Write{}, 0x38, 0xff_B}); // gradation 0-7
        spi::put(Command{Write{}, 0x3a, 0b0000'0100_B}); // groups

        spi::put(Command{Write{}, 0x3e, 0x0f_B}); // start 0, 1 contious

        spi::put(Command{Write{}, 0x2c, 0xc1_B}); // ramp up/down
        spi::put(Command{Write{}, 0x2d, 0x02_B}); // step time
        spi::put(Command{Write{}, 0x2e, 0xc9_B}); // hold 
        spi::put(Command{Write{}, 0x2f, 100_B}); // iref
        
        spi::put(Command{Write{}, 2, 0b0000'0101_B});            
        
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

template<typename Timer, typename Term, typename SW, AVR::Concepts::Pin Led>
struct GFSM {
    using terminal = Term;
    using terminal_device = typename Term::device_type;
    using pca = Pca;
    using spi = pca::spi;

    using led = AVR::ActiveHigh<Led>;
    using blinker = External::Blinker2<led, Timer, 100_ms, 2000_ms>;
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> debugTicks{500_ms};

    enum class State : uint8_t {Undefined, 
                                Init, Run,
                                StartWait, SearchChannel, AfterSearch, 
                                ShowAddress, ShowAddressWait, LearnTimeout,
                               };
    
    static inline void init() {
        terminal_device::template init<AVR::BaudRate<115200>>();
        pca::init();        
        blinker::init();    
    }    
    
    static inline void periodic() {
        terminal_device::periodic();
        pca::periodic();        
    }    
    static inline void ratePeriodic() {
        blinker::ratePeriodic();
        const auto oldState = mState;
        ++mStateTicks;
        switch(mState) {
        case State::Undefined:
            mState = State::StartWait;
            blinker::steady();
            break;
        case State::StartWait:
            mStateTick.on(waitTimeoutTicks, []{
                blinker::off();
                mState = State::SearchChannel;
            });
            break;
        case State::SearchChannel:
            if (search()) {
                mState = State::AfterSearch;
            }
            mStateTick.on(learnTimeoutTicks, []{
                mState = State::LearnTimeout;
            });
            break;
        case State::AfterSearch:
            mStateTick.on(signalTimeoutTicks, []{
                mState = State::ShowAddress;
            });
            break;
        case State::LearnTimeout:
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(typename SW::addr_t{NVM::data().address().toInt()});
            }
            mState = State::InitRun;
            break;
        case State::ShowAddress:
            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            blinker::onePeriod(bl_count_t(data().address().toInt() + 1));
            mState = State::ShowAddressWait;
            break;
        case State::ShowAddressWait:
            if (!blinker::isActive()) {
                mState = State::InitRun;
            }
            break;
        case State::Undefined:
            mStateTicks.on(startupTicks, []{
               mState = State::Init; 
            });
            break;
        case State::Init:
            mStateTicks.on(startupTicks, []{
               mState = State::Run; 
            });
            break;
        case State::Run:
            mStateTicks.on(debugTicks, debug);
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("Init"_pgm);
                break;
            case State::Run:
                etl::outl<terminal>("Run"_pgm);
                break;
            }
        }
        
    }
private:
    inline static void debug() {
        etl::outl<terminal>("c: "_pgm, ++mCounter, " sc: "_pgm, spi::count());
        if (mCounter & 0x01) {
            pca::template out<true>(0);
        }
        else {
            pca::template out<false>(0);
        }
    }
    static inline uint16_t mCounter{};

    
    using addr_t = typename protocol_t::addr_t;
    
    static inline bool search() {
        if (learnChannel < 16) {
            if (const auto lc = protocoll_adapter_t::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
                etl::outl<Term>("ch: "_pgm, learnChannel.toInt(), " : "_pgm, lc.toInt());
                if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= SW::protocol_t::addr_t::Upper)) {
                    const uint8_t addr = pv - 1;
                    SW::channel(learnChannel.toRangedNaN());
                    SW::address(addr_t(addr));
                    NVM::data().channel() = learnChannel;
                    NVM::data().address() = addr;
                    NVM::data().change();
                    return true;
                }
            }   
        }
#ifdef LEARN_DOWN
        --learnChannel;
#else
        ++learnChannel;
#endif
        return false;
    }
#ifdef LEARN_DOWN
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{ch_t::Upper};
#else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
#endif
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mEepromTick;
    static inline External::Tick<Timer> mStateTicks;
    static inline State mState{State::Undefined};
};

template<auto N = 8>
struct SwitchStates {
    enum class SwState : uint8_t {Off, Blink1, Steady, Blink2, PassThru};
    
    static constexpr void init() {
    }
    static constexpr uint8_t size() {
        return N;
    }
    static inline auto& switches() {
        return swStates;
    }
    static inline void testMode(const auto& v) {
//        decltype(v)::_;
        if (const auto vv = v.toInt(); vv == 0) {
            std::fill(std::begin(swStates), std::end(swStates), SwState::Off);
        }
        else if ((vv >= 1) && (vv <= 8)) {
            std::fill(std::begin(swStates), std::end(swStates), SwState::Off);
            swStates[vv - 1] = SwState::Steady;
        }
        else {
            std::fill(std::begin(swStates), std::end(swStates), SwState::Steady);
        }
        
    }
private:
    static inline std::array<SwState, N> swStates{};
};


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

using servo_pa = IBus::Servo::ProtocollAdapter<0>;

using terminal_device = Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<terminal_device>;

using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;

template<typename Command>
using spi = AVR::SpiSS<spiPosition, Command, AVR::QueueLength<16>, csPin>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, spiPosition>>;

using pca9745 = PCA9745<spi, oePin, iref1Pin, iref2Pin>;

using sw = SwitchStates<>;
using ibus_switch = IBus::Switch::Digital<servo_pa, sw, void>;

using gfsm = GFSM<systemTimer, terminal, ibus_switch, ledPin>;

int main() {
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();

    gfsm::init();
    
    etl::outl<terminal>("test10"_pgm);
    while(true) {
        gfsm::periodic();
        systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
//        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
