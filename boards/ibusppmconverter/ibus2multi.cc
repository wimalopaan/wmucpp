#define USE_IBUS

#include "board.h"

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

template<typename PPM, uint8_t Size = 8>
struct FSM {
    enum class SwState : uint8_t {Off, Steady, Blink1, Blink2};
    
    using ppm_index_t = PPM::index_type;
    using ppm_value_t = PPM::ranged_type;

    using cycle_t = etl::uint_ranged_circular<uint8_t, 0, 9>;
    
    static inline constexpr uint8_t offset{10};
    static inline etl::uint_ranged_circular<uint8_t, 0, ppm_index_t::Upper> index;

    static inline void init() {
        PPM::init();    
    }
    static inline void periodic() {
        PPM::template onCompareMatch<PWM::WO<0>>([&]{
            update<0>();
        });
        PPM::template onCompareMatch<PWM::WO<1>>([&]{
            update<1>();
        });
        PPM::template onCompareMatch<PWM::WO<2>>([&]{
            update<2>();
        });
    }
    static inline auto& switches() {
        return swStates[0];
    }
private:
    template<uint8_t channel>
    inline static void update() {
        if (cycles[channel] < 2) {
            PPM::ppmRaw(ppm_index_t{channel}, PPM::ocMax + 100);
        }
        else {
            const uint8_t i = cycles[channel] - 2;
            if (swStates[channel][i] == SwState::Off) {
                PPM::ppm(ppm_index_t{channel}, ppm_value_t{PPM::ocMedium});
            }
            else if (swStates[channel][i] == SwState::Steady) {
                PPM::ppm(ppm_index_t{channel}, ppm_value_t{PPM::ocMax - 200});
            }
            else if (swStates[channel][i] == SwState::Blink1) {
                PPM::ppm(ppm_index_t{channel}, ppm_value_t{PPM::ocMin + 200});
            }
        }
        ++cycles[channel];
    }
    static inline std::array<cycle_t, ppm_index_t::Upper>  cycles;
    static inline std::array<std::array<SwState, Size>, ppm_index_t::Upper> swStates;
};

using fsm = FSM<ppmA>;
using ibus_switch = IBus::Switch::Switch4<servo_pa, fsm>;

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
    
    servo::init<AVR::BaudRate<115200>, HalfDuplex>();
    servo::txEnable<false>();
    
    fsm::init();
    ibus_switch::init();
    
    const auto periodicTimer = alarmTimer::create(20_ms, External::Hal::AlarmFlags::Periodic);

    while(true) {
        servo::periodic();
        ibus_switch::periodic();
        fsm::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
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
