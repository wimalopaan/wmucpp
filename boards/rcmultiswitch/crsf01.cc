#define NDEBUG
#define AUTO_BUS

#include "board.h"
#include "crsf.h"

template<typename ComCb>
struct Devices {
    struct CrsfAdapterConfig;
    using crsf_pa = Crsf::Adapter<CrsfAdapterConfig>;
    using crsf = AVR::Usart<usart0Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminalDevice = crsf;
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CrsfAdapterConfig {
        using debug = terminal;
        using cb = ComCb;
    };

    static inline void init() {
        crsf::template init<BaudRate<420000>>();

        led0::dir<AVR::Output>();
        led1::dir<AVR::Output>();
        led2::dir<AVR::Output>();
        led3::dir<AVR::Output>();
        led4::dir<AVR::Output>();
        led5::dir<AVR::Output>();
        led6::dir<AVR::Output>();
        led7::dir<AVR::Output>();
    }
    static inline void periodic() {
        crsf::periodic();
    }
    static inline void ratePeriodic() {
        crsf_pa::ratePeriodic();
    }
};

struct CrsfCommandCallback {
    static inline void set(const std::byte data) {
        if (std::any(data & 0x01_B)) {
            led0::on();
        }
        else {
            led0::off();
        }
        if (std::any(data & 0x02_B)) {
            led1::on();
        }
        else {
            led1::off();
        }
        if (std::any(data & 0x04_B)) {
            led2::on();
        }
        else {
            led2::off();
        }
        if (std::any(data & 0x08_B)) {
            led3::on();
        }
        else {
            led3::off();
        }
        if (std::any(data & 0x10_B)) {
            led4::on();
        }
        else {
            led4::off();
        }
        if (std::any(data & 0x20_B)) {
            led5::on();
        }
        else {
            led5::off();
        }
        if (std::any(data & 0x40_B)) {
            led6::on();
        }
        else {
            led6::off();
        }
        if (std::any(data & 0x80_B)) {
            led7::on();
        }
        else {
            led7::off();
        }
    }
};

using devices = Devices<CrsfCommandCallback>;

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

