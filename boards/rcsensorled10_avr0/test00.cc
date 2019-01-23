#define NDEBUG

#include "board.h"

using terminal = etl::basic_ostream<rcUsart>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    systemClock::init();
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();

    rcUsart::init<115200>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;
        
        while(true) {
//            ppmInPin::toggle();
            rcUsart::periodic();
            btUsart::periodic();
            systemClock::periodic([&](){
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        outl<terminal>("test"_pgm);    
                    }
                });
            });
        }    
    }
}
