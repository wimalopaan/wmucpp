#define NDEBUG

#include "devices.h"

using devs = Devices<0>;
using portmux = devs::portmux;
using ccp = devs::ccp;
using clock = devs::clock;
using systemTimer = devs::systemTimer;

int main() {
//        portmux::init();
//        ccp::unlock([]{
//                clock::template init<Project::Config::fMcuMhz>();
//        });
        systemTimer::init(); 
//    while(true) {
//        systemTimer::periodic([&]{
//        });
//    }
}

