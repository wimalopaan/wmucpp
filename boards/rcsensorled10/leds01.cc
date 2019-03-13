#define NDEBUG

#include "local.h"
#include "rcsensorled10.h"
#include "console.h"

using rcUsart = AVR::Usart<1, void , MCU::UseInterrupts<false>, UseEvents<false>,AVR::ReceiveQueueLength<64>, AVR::SendQueueLength<64>>;

using terminalDevice = rcUsart;
using terminal = std::basic_ostream<terminalDevice>;

constexpr auto sequence = "1r;2r;p10";

struct SequenceItem {
    std::byte a;
    std::byte b;
};

constexpr auto parse(auto s) {
    std::array<SequenceItem, 100> seq;
    
//    for(const char )    
    
    
    return seq;
}

constexpr auto x = parse(sequence);

struct Leds {
    
};


int main() {
    constexpr std::hertz fCr = 1 / Hott::hottDelayBetweenBytes;
    static_assert(fCr == Config::Timer::frequency);
    
    leds::init();
    
    alarmTimer::init(AVR::TimerMode::CTCNoInt); 
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        std::outl<terminal>("led01"_pgm);
        Util::delay(100_ms);
        
        const auto periodicTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
        
        Util::delay(100_ms);
        
        while(true) {
            rcUsart::periodic();
            systemClock::periodic<systemClock::flags_type::ocfa>([&](){
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *periodicTimer) {
                    }
                });
            });
        }
    }
}
