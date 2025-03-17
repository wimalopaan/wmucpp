#define NDEBUG
#define AUTO_BUS

#include "board.h"

template<typename Config>
struct SBusCommandCallback {
    using debug = Config::debug;

    static inline uint8_t lastCommand = 0;

    static inline void decode(const std::array<uint8_t, 23>& data) {
        const uint16_t ch16 = (uint16_t) ((data[20]>>5 | data[21]<<3) & 0x07FF);
        const uint16_t n = (ch16 >= 172) ? (ch16 - 172) : 0;
        const uint8_t  v = (n >> 4);

        const uint8_t address = (v >> 4) & 0b11;
        const uint8_t sw      = (v >> 1) & 0b111;
        const uint8_t state   = v & 0b1;

        if (v != lastCommand) {
            lastCommand = v;
            etl::outl<debug>("v: "_pgm, v, etl::Char{' '}, address, etl::Char{' '}, sw, etl::Char{' '}, state);
            set(sw, (state == 1));
        }
    }
    static inline void set(const uint8_t led, const bool on) {
        if (led == 0) {
            if (on) {
                led0::on();
            }
            else {
                led0::off();
            }
        }
        else if (led == 1) {
            if (on) {
                led1::on();
            }
            else {
                led1::off();
            }
        }
        else if (led == 2) {
            if (on) {
                led2::on();
            }
            else {
                led2::off();
            }
        }
        else if (led == 3) {
            if (on) {
                led3::on();
            }
            else {
                led3::off();
            }
        }
        else if (led == 4) {
            if (on) {
                led4::on();
            }
            else {
                led4::off();
            }
        }
        else if (led == 5) {
            if (on) {
                led5::on();
            }
            else {
                led5::off();
            }
        }
        else if (led == 6) {
            if (on) {
                led6::on();
            }
            else {
                led6::off();
            }
        }
        else if (led == 7) {
            if (on) {
                led7::on();
            }
            else {
                led7::off();
            }
        }
    }
};

template<typename Config>
struct Devices {
    struct CallbackConfig;

    using cb = SBusCommandCallback<CallbackConfig>;

    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer, void, cb>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminalDevice = servo;
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CallbackConfig {
        using debug = terminal;
    };

    static inline void init() {
        led0::dir<AVR::Output>();
        led1::dir<AVR::Output>();
        led2::dir<AVR::Output>();
        led3::dir<AVR::Output>();
        led4::dir<AVR::Output>();
        led5::dir<AVR::Output>();
        led6::dir<AVR::Output>();
        led7::dir<AVR::Output>();

        servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
        servo::rxInvert(true);
    }
    static inline void periodic() {
        servo::periodic();
    }

    static constexpr External::Tick<systemTimer> debugTicks{500_ms};

    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        ++mStateTicks;

        mStateTicks.on(debugTicks, []{
            etl::outl<terminal>("x"_pgm);
        });
    }

    inline static External::Tick<systemTimer> mStateTicks;

};

struct DevsConfig {
};
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

