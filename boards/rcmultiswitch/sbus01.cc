#define NDEBUG
#define AUTO_BUS

#define DEFAULT_ADDRESS 0

#define USE_ELRS
//#define USE_AFHDS2A
//#define USE_FRSKY

#include "board.h"
#include "leds.h"

template<typename Config>
struct SBusCommandCallback {
    using debug = Config::debug;
    using leds = Config::leds;

    static inline void decode(const std::array<uint8_t, 23>& data) {
        const uint16_t ch16 = (uint16_t) ((data[20]>>5 | data[21]<<3) & 0x07FF);
#ifdef USE_ELRS
        static constexpr uint16_t value0 = 172;
        const uint16_t n = (ch16 >= value0) ? (ch16 - value0) : 0;
        const uint8_t  v = (n >> 4);
#endif
#ifdef USE_AFHDS2A
        static constexpr uint16_t value0 = 220;
        const uint16_t n = (ch16 >= value0) ? (ch16 - value0) : 0;
        const uint16_t n2 = n + (n >> 6);
        const uint8_t  v = (n2 >> 4);
#endif
        const uint8_t address = (v >> 4) & 0b11;
        const uint8_t sw      = (v >> 1) & 0b111;
        const uint8_t state   = v & 0b1;

        if (v != mLastCommand) {
            mLastCommand = v;
            etl::outl<debug>("ch16: "_pgm, ch16, ", adr: "_pgm, address, " sw: "_pgm, sw, ", state: "_pgm, state);
            if (address == mAddress) {
                const bool on = (state == 1);
                leds::set(sw, on);
            }
        }
    }
    private:
    static inline uint8_t mLastCommand = 0;
    static inline uint8_t mAddress = DEFAULT_ADDRESS;
};

template<typename Config>
struct Devices {
    struct CallbackConfig;

    using leds = Leds<ledList>;

    using cb = SBusCommandCallback<CallbackConfig>;

    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer, void, cb>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminalDevice = servo;
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CallbackConfig {
        using debug = terminal;
        using leds = Devices::leds;
    };
    static inline void init() {
        leds::init();
        servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
        servo::rxInvert(true);
    }
    static inline void periodic() {
        servo::periodic();
    }
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        ++mStateTicks;
        mStateTicks.on(debugTicks, []{
            etl::outl<terminal>("tick"_pgm);
        });
    }
    private:
    static constexpr External::Tick<systemTimer> debugTicks{500_ms};
    inline static External::Tick<systemTimer> mStateTicks;
};

struct DevsConfig {};
using devices = Devices<DevsConfig>;

int main() {
    portmux1::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();

    devices::init();
    while(true) {
        devices::periodic();
        systemTimer::periodic([&]{
            devices::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<devices::terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
   while(true) {
//        dbg1::toggle();
   }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif

