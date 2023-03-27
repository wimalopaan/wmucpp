#include "devices.h"

// Purpose:
// Toggle la0 with maximum frequency
// Toggle la1 with rate 1000Hz

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    
    static void init() {
        devs::init();
    } 
    static void periodic() {
        devs::la0::toggle();
    } 
    static void ratePeriodic() {
        devs::la1::toggle();        
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
