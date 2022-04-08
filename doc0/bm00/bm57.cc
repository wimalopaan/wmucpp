#include <mcu/avr.h>

#define MORE
//#define ENTER

volatile uint8_t r = 0;
volatile uint8_t x = 0;

namespace {
    struct FSM {
        enum class State : uint8_t {Test0, Test1, Test2, Test3, Test4
                        #ifdef MORE
                                    , Test5, Test6, Test7
                        #endif
                                   };
        
        inline static void process(const uint8_t b) {
            const auto oldState = mState;
            if (mState == State::Test0) {
                if (b == 0) {
                    mState = State::Test1;                
                }
            }
            else if (mState == State::Test1) {
                if (b == 1) {
                    mState = State::Test2;
                }
            }
            else if (mState == State::Test2) {
                if (b == 2) {
                    mState = State::Test3;
                }
            }
            else if (mState == State::Test3) {
                if (b == 3) {
                    mState = State::Test4;
                }
            }
            else if (mState == State::Test4) {
                if (b == 4) {
#ifndef MORE
                    mState = State::Test0;                                
#else
                    mState = State::Test5;                                
                }
            }
            else if (mState == State::Test5) {
                if (b == 5) {
                    mState = State::Test6;
                }
            }
            else if (mState == State::Test6) {
                if (b == 6) {
                    mState = State::Test7;
                }
            }
            else if (mState == State::Test7) {
                if (b == 7) {
                    mState = State::Test0;
                }
            }
#endif
#ifdef ENTER
        if (oldState != mState) {
            if (mState == State::Test1) {
                x = 41;                    
            }
            else {
                if (mState == State::Test2) {
                    x = 42;                        
                }
                else {
                    if (mState == State::Test3) {
                        x = 43;                            
                    }
                    else {
                        if (mState == State::Test4) {
                            x = 44;                                
                        }
#ifdef MORE
                        else {
                            if (mState == State::Test5) {
                                x = 45;                                    
                            }
                            else {
                                if (mState == State::Test6) {
                                    x = 46;                                        
                                }
                                else {
                                    if (mState == State::Test7) {
                                        x = 47;                                        
                                    }                                        
                                }
                            }
                        }
#endif
                    }
                }
            }
        }
#endif
    }
    private:
    inline static State mState{State::Test0};
};

using fsm = FSM;
}

int main() {
    while(true) {
        const uint8_t b = r;        
        fsm::process(b);
    }
}
