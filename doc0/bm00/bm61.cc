#include <mcu/avr.h>

volatile uint8_t r = 0;
volatile uint8_t x = 0;

namespace {
    struct FSM {
        enum class State : uint8_t {Off, Start, Running, Test0, Test1, Test2, Test3, Test4, Test5, Test6, Test7, Test8, Test9, Error};
        
//        inline static void periodic() {
//            process(r);
//        }
        inline static void process(const uint8_t b) {
            const auto oldState = mState;
            switch(mState) {
            case State::Off:
                if (b == 1) {
                    mState = State::Start;
                }
                break;
            case State::Start:
                if (b == 3) {
                    mState = State::Test0;
                }
                else if (b == 0) {
                    mState = State::Error;
                }
                break;
            case State::Test0:
                if (b == 0) {
                    mState = State::Test1;
                }
                break;
            case State::Test1:
                if (b == 1) {
                    mState = State::Test2;
                }
                break;
            case State::Test2:
                if (b == 2) {
                    mState = State::Test3;
                }
                break;
            case State::Test3:
                if (b == 3) {
                    mState = State::Test4;
                }
                break;
            case State::Test4:
                if (b == 4) {
                    mState = State::Test5;
                }
                break;
            case State::Test5:
                if (b == 5) {
                    mState = State::Test6;
                }
                break;
            case State::Test6:
                if (b == 6) {
                    mState = State::Test7;
                }
                break;
            case State::Test7:
                if (b == 7) {
                    mState = State::Test8;
                }
                break;
            case State::Test8:
                if (b == 8) {
                    mState = State::Test9;
                }
                break;
            case State::Test9:
                if (b == 9) {
                    mState = State::Running;
                }
                break;
            case State::Running:
                if (b == 3) {
                    mState = State::Off;
                }
                else if (b > 100) {
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
                case State::Test0:
                    x = 40;
                    break;
                case State::Test1:
                    x = 41;
                    break;
                case State::Test2:
                    x = 42;
                    break;
                case State::Test3:
                    x = 43;
                    break;
                case State::Test4:
                    x = 44;
                    break;
                case State::Test5:
                    x = 45;
                    break;
                case State::Test6:
                    x = 46;
                    break;
                case State::Test7:
                    x = 47;
                    break;
                case State::Test8:
                    x = 48;
                    break;
                case State::Test9:
                    x = 49;
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
        inline static State mState = State::Off;
    };
}

using fsm = FSM;

int main() {
    while(true) {
        const uint8_t b = r;        
        fsm::process(b);
    }
}
