#define NDEBUG

#define USE_IBUS

#include "board.h"
#include "swout.h"

using servo_pa = IBus::Servo::ProtocollAdapter<0>;

using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
using terminalDevice = servo;
using terminal = etl::basic_ostream<servo>;

template<auto N = 8>
struct SwitchStates {
    enum class SwState : uint8_t {Off, On, Blink1 = On, Steady, Blink2, PassThru};
    
    static constexpr void init() {
    }
    static constexpr uint8_t size() {
        return N;
    }
    static inline auto& switches() {
        return swStates;
    }
private:
    static inline std::array<SwState, N> swStates{};
};

using sw = SwitchStates<>;

using out = External::Output<ledList, pwm, sw, eeprom>;

using ibus_switch = IBus::Switch::Switch3<servo_pa, sw, out>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    servo::init<BaudRate<115200>>();
    
    systemTimer::init();

    out::init();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    etl::outl<terminal>("multi8d"_pgm);

    uint16_t counter{};
    
    using ch_t = servo_pa::channel_t;
    
    while(true) {
        servo::periodic();
        
        systemTimer::periodic([&]{
            ibus_switch::periodic();
            out::setSwitches();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("c: "_pgm, counter++, " ch: "_pgm, ibus_switch::lv, 
//                                        " gc: "_pgm, ibus_switch::gc, " gc2: "_pgm, ibus_switch::gc2
                                        " lp: "_pgm, ibus_switch::lp,
                                        " lp2: "_pgm, ibus_switch::lp2
                                        );
                }
            });
        });
    }
}

