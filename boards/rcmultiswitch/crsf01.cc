#define NDEBUG
#define AUTO_BUS

#include "board.h"
#include "crsf.h"
#include "leds.h"

template<typename Leds>
struct CrsfCommandCallback {
    static inline void set(const std::byte data) {
        std::byte mask = 0b1_B;
        for(uint8_t i = 0; i < 8; ++i) {
            if (std::any(data & mask)) {
                Leds::set(i, true);
            }
            else {
                Leds::set(i, false);
            }
            mask <<= 1;
        }
    }
};

template<typename Config>
struct Devices {
    using leds = Leds<ledList>;

    struct CrsfAdapterConfig;
    using crsf_pa = Crsf::Adapter<CrsfAdapterConfig>;
    using crsf = AVR::Usart<usart0Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminalDevice = crsf;
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CrsfAdapterConfig {
        using debug = terminal;
        using cb = CrsfCommandCallback<leds>;
    };

    static inline void init() {
        leds::init();
        crsf::template init<BaudRate<420000>>();
    }
    static inline void periodic() {
        crsf::periodic();
    }
    static inline void ratePeriodic() {
        crsf_pa::ratePeriodic();
        ++mStateTicks;
        mStateTicks.on(debugTicks, []{
            etl::outl<terminal>("cp: "_pgm, crsf_pa::commandPackages());
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

