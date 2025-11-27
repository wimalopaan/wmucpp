#pragma once

#include "mcu/mcu.h"

template<typename Config>
struct Scheduler {
    using fsm = Config::fsm;
    using timer = Config::timer;
    [[noreturn]]static inline void main(auto irqEnable) {
        fsm::init();
		if constexpr(requires(){fsm::update(true);}) {
			fsm::update(true);
		}
        irqEnable();
        __enable_irq();
        while(true) {
            fsm::periodic();
            timer::periodic([]{
                fsm::ratePeriodic();
            });
        }
    }
};
