#define USE_IBUS

#define ROBBE_8370

#ifdef ROBBE_8370
//# define HAS_MEMORY_FUNCTION
#endif

#include "board.h"

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;

using tcb0Position = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppmB = External::Ppm::PpmOut<tcb0Position>;

using tcb1Position = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
using ppmC = External::Ppm::PpmOut<tcb1Position>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position, tcb1Position>>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

template<typename PPM, uint8_t Channel = 0>
struct Adapter {
    using ppm_index_t = PPM::index_type;
    
    inline static constexpr auto ocMax =  PPM::ocMax;
    inline static constexpr auto ocMin =  PPM::ocMin;
    inline static constexpr auto ocMedium =  PPM::ocMedium;
    
    inline static void init() {
        PPM::init();
    }
    using ranged_type = PPM::ranged_type;    
    inline static void onReload(auto f) {
        PPM::template onCompareMatch<PWM::WO<Channel>>([&]{
            f();
        });
    }
    
    inline static void ppmRaw(const auto v) {
        PPM::ppmRaw(ppm_index_t{Channel}, v);
    }

    inline static void ppm(const auto v) {
        PPM::ppm(ppm_index_t{Channel}, v);
    }
};


template<typename PPM, typename PA, typename NVM, uint8_t Address = 0, uint8_t Size = 8>
struct FSM {
    inline static constexpr uint8_t address = Address;
    
    enum class SwState : uint8_t {Off, Steady, Blink1, Blink2};

    using ppm_value_t = PPM::ranged_type;

#ifdef ROBBE_8370
    using cycle_t = etl::uint_ranged_circular<uint8_t, 0, 8>;
#else 
    using cycle_t = etl::uint_ranged_circular<uint8_t, 0, 9>;
#endif
    
    static inline constexpr uint8_t offset{10};
    
    static inline void init() {
        PPM::init();    
    }
    static inline void periodic() {
        PPM::onReload([]{
            update();
        });
    }
    static inline auto& switches() {
        return swStates;
    }
//private:
    inline static void update() {
#ifdef ROBBE_8370
        if (cycle < 1) {
            PPM::ppmRaw(PPM::ocMin - 100);
        }
#else
        if (cycle < 2) {
            PPM::ppmRaw(PPM::ocMax + 100);
        }
#endif
        else {
#ifdef ROBBE_8370
            const uint8_t i = Size - 1 - (cycle - 1); // Robbe counts channels in reverse order
#else
            const uint8_t i = cycle - 2;
#endif
            if (const auto pch = NVM::data()[i].passThru()) {
                using ch_t = PA::channel_t;
                auto v = PA::value(ch_t{pch});
//                decltype(v)::_;
                PPM::ppm(v);
            }
            else if (swStates[i] == SwState::Off) {
                PPM::ppmRaw(PPM::ocMedium);
            }
#ifdef HAS_MEMORY_FUNCTION
            else if (swStates[i] != swStatesLast[i]) {
                swStatesLast[i] = swStates[i];
                if (swStates[i] == SwState::Steady) {
                    PPM::ppmRaw(PPM::ocMax - 200);
                }
                else if (swStates[i] == SwState::Blink1) {
                    PPM::ppmRaw(PPM::ocMin + 200);
                }
            }
            else {
                PPM::ppmRaw(PPM::ocMedium);
            }
#else
            else if (swStates[i] == SwState::Steady) {
                PPM::ppmRaw(PPM::ocMax - 200);
            }
            else if (swStates[i] == SwState::Blink1) {
                PPM::ppmRaw(PPM::ocMin + 200);
            }
#endif
        }
        ++cycle;
    }
    static inline cycle_t cycle;
    static inline std::array<SwState, Size> swStates;
#ifdef HAS_MEMORY_FUNCTION
    static inline std::array<SwState, Size> swStatesLast;
#endif
};

using ppmCh1 = Adapter<ppmA, 0>;
using ppmCh2 = Adapter<ppmA, 1>;
using ppmCh3 = Adapter<ppmA, 2>;
using fsm1 = FSM<ppmCh1, servo_pa, eeprom, 0>;
using fsm2 = FSM<ppmCh2, servo_pa, eeprom, 1>;
using fsm3 = FSM<ppmCh3, servo_pa, eeprom, 2>;
using fsm4 = FSM<ppmB, servo_pa, eeprom, 3>;
using fsm5 = FSM<ppmC, servo_pa, eeprom, 4>;

using ibus_switch = IBus::Switch::MultiAdapter<servo_pa, Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5>, eeprom>;

using evch0 = Event::Channel<0, void>;
using evch1 = Event::Channel<1, void>;
using evuser0 = Event::Route<evch0, Event::Users::Tcb<0>>;
using evuser1 = Event::Route<evch1, Event::Users::Tcb<1>>;
using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;

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
    
    evrouter::init();
    portmux::init();
    systemTimer::init();
    
    servo::init<AVR::BaudRate<115200>, HalfDuplex>();
    servo::txEnable<false>();
    
    fsm1::init();
    fsm2::init();
    fsm3::init();
    fsm4::init();
    fsm5::init();
    ibus_switch::init();
    
    const auto periodicTimer = alarmTimer::create(20_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        servo::periodic();
        ibus_switch::periodic();
        fsm1::periodic();
        fsm2::periodic();
        fsm3::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ppmB::onReload([]{
                        fsm4::update();
                        evrouter::strobe<0>();
                    });
                    ppmC::onReload([]{
                        fsm5::update();
                        evrouter::strobe<1>();
                    });
                }
            });
        });
    }
}

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
