#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    
    static void init() {
        devs::init();
        
        devs::blinkLed1::init();
        devs::blinkLed2::init();

        using blink1_t = devs::blinkLed1::count_type;
        devs::blinkLed1::blink(blink1_t{4});

        using blink2_t = devs::blinkLed2::count_type;
        devs::blinkLed2::blink(blink2_t{2});
        
        devs::buzz::init();
    } 
    static void periodic() {
        devs::la0::toggle();
    } 
    static void ratePeriodic() {
        devs::la1::toggle();        

        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();
        
        devs::buzz::toggle();
    }     
};

using devices = Devices<1>;
using gfsm = GlobalFsm<devices>;

int main() {
    gfsm::init();    
    while(true) {
        gfsm::periodic(); 
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
    
}
