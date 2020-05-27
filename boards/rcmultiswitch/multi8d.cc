#define NDEBUG

#include "board.h"
#include "swout.h"

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using ibus_switch = IBus::Switch::Switch2<servo_pa>;

using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
using terminalDevice = servo;
using terminal = etl::basic_ostream<servo>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    servo::init<BaudRate<115200>>();
    
    systemTimer::init();

    Meta::visit<ledList>([]<typename L>(Meta::Wrapper<L>){
                             L::template dir<Output>();
                         });

    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    etl::outl<terminal>("multi8d"_pgm);

    uint16_t counter{};
    
    while(true) {
        servo::periodic();
        
        systemTimer::periodic([&]{
            ibus_switch::periodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("c: "_pgm, counter++);
                }
            });
        });
    }
}

