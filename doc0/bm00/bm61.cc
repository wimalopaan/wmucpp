#include <mcu/avr.h>

volatile uint8_t r;
volatile uint8_t x;

namespace {
    struct FSM {
        enum class State : uint8_t {Off, Start, Running, Error};
        
        inline static void periodic() {
            process(r);
        }
        inline static void process(uint8_t value) {
            auto oldState = mState;
            switch(mState) {
            case State::Off:
                if (value == 1) {
                    mState = State::Start;
                }
                break;
            case State::Start:
                if (value == 3) {
                    mState = State::Running;
                }
                break;
            case State::Running:
                if (value == 3) {
                    mState = State::Off;
                }
                else if (value > 100) {
                    mState = State::Error;
                }
                break;
            case State::Error:
                break;
            }
            if (oldState != mState) {
                switch(mState) {
                case State::Off:
                    break;
                case State::Start:
                    x = 43;
                    break;
                case State::Running:
                    x = 44;
                    break;
                case State::Error:
                    break;
                }
            }
        }
    private:
        inline static constinit State mState = State::Off;
    };
}

using fsm = FSM;

int main() {
    while(true) {
        fsm::periodic();
    }
}
