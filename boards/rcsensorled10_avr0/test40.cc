#define NDEBUG

#include "board.h"

// spi-pins als debug
//using testPin1 = AVR::Pin<PortC, 4>; 
//using testPin2 = AVR::Pin<PortC, 5>; 
//using testPin3 = AVR::Pin<PortC, 6>; 
//using testPin4 = AVR::Pin<PortC, 7>; 


#include <external/hott/experimental/sensor.h>

using sensor = Hott::Experimental::Sensor<0, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, systemClock>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
//    testPin1::dir<AVR::Output>();
//    testPin1::off();
//    testPin2::dir<AVR::Output>();
//    testPin2::off();
//    testPin3::dir<AVR::Output>();
//    testPin3::off();
//    testPin4::dir<AVR::Output>();
//    testPin4::off();
    
    sensor::init();
    
    systemClock::init();
    
    rcUsart::init<BaudRate<115200>>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    outl<terminal>("test40"_pgm);    
    {
        Scoped<EnableInterrupt<>> ei;
        
        while(true) {
            rcUsart::periodic();
            
            sensor::periodic();
            
            systemClock::periodic([&](){
                
                sensor::ratePeriodic();
                
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        outl<terminal>("ar/br/coll "_pgm, sensor::asciiPackages(), Char{','}, sensor::binaryPackages(), Char{','}, sensor::collisions());    
                    }
                });
            });
        }    
    }
}
