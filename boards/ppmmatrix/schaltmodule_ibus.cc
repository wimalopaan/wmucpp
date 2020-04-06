#define NDEBUG

#define SCHALTMODUL_IBUS

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

using terminal = etl::basic_ostream<terminalDevice>;
using components = AVR::Components<rcUsart, terminalDevice, cppm>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    systemClock::init();
    cppm::init();
    
    rxSelect::dir<Output>();
    rxSelect::off();
    
    paired::dir<Input>();
    
    rcUsart::init<AVR::BaudRate<115200>>();
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    using channel_t = servo_pa::channel_t;
    using out_t = cppm::index_type;
    {
        Scoped<EnableInterrupt<>> ei;
        
        etl::outl<terminal>("Schaltmodul Ibus 01"_pgm);    

        etl::uint_ranged_circular<uint8_t, 0, out_t::Upper> index{};
        
        while(true) {
            components::periodic();
            ++index;
            auto v = servo_pa::value(channel_t(10 + index));
            cppm::ppm(index, v);
            systemClock::periodic([&](){
                alarmTimer::periodic([&](const alarmTimer::index_type timer){
                    if (timer == t) {
                        auto v = servo_pa::value(channel_t{14});
                        
                        etl::outl<terminal>("v15: "_pgm, v.toInt());    
                    }
                });
            });
        }    
    }
}

