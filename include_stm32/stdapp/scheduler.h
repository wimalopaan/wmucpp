#pragma once

template<typename Config>
struct Scheduler {
    private:
    using fsm = Config::fsm;
    using timer = Config::timer;
    public:
    [[noreturn]]static inline void main(auto irqEnable) {
        fsm::init();
        fsm::update(true);

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
