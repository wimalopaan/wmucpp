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
            if (mState == State::Off) {
                if (b == 1) {
                    mState = State::Start;
                }
            }
            else {
                if (mState == State::Start) {
                    if (b == 3) {
                        mState = State::Test0;
                    }
                    else if (b == 0) {
                        mState = State::Error;
                    }
                }
                else {
                    if (mState == State::Test0) {
                        if (b == 0) {
                            mState = State::Test1;
                        }
                    }
                    else {
                        if (mState == State::Test1) {
                            if (b == 1) {
                                mState = State::Test2;
                            }
                        }
                        else {
                            if (mState == State::Test2) {
                                if (b == 2) {
                                    mState = State::Test3;
                                }
                            }
                            else {
                                if (mState == State::Test3) {
                                    if (b == 3) {
                                        mState = State::Test4;
                                    }
                                }
                                else {
                                    if (mState == State::Test4) {
                                        if (b == 4) {
                                            mState = State::Test5;
                                        }
                                    }
                                    else {
                                        if (mState == State::Test5) {
                                            if (b == 5) {
                                                mState = State::Test6;
                                            }
                                        }
                                        else {
                                            if (mState == State::Test6) {
                                                if (b == 6) {
                                                    mState = State::Test7;
                                                }
                                            }
                                            else {
                                                if (mState == State::Test7) {
                                                    if (b == 7) {
                                                        mState = State::Test8;
                                                    }
                                                }
                                                else {
                                                    if (mState == State::Test8) {
                                                        if (b == 8) {
                                                            mState = State::Test9;
                                                        }
                                                    }
                                                    else {
                                                        if (mState == State::Test9) {
                                                            if (b == 9) {
                                                                mState = State::Running;
                                                            }
                                                        }
                                                        else {
                                                            if (mState == State::Running) {
                                                                if (b == 3) {
                                                                    mState = State::Off;
                                                                }
                                                                else if (b > 100) {
                                                                    mState = State::Error;
                                                                }
                                                            }
                                                            else {
                                                                if (mState == State::Error) {
                                                                }
                                                                else {
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (oldState != mState) {
                if (mState == State::Start) {
                    x = 43;
                }
                else {
                    if (mState == State::Test0) {
                        x = 40;                    
                    }
                    else {
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
                                                else {
                                                    if (mState == State::Test8) {
                                                        x = 48;                    
                                                    }
                                                    else {
                                                        if (mState == State::Test9) {
                                                            x = 49;                    
                                                        }
                                                        else {
                                                            if (mState == State::Error) {
                                                                x = 44;
                                                            }
                                                            else {
                                                                if (mState == State::Running) {
                                                                }
                                                                else {
                                                                    if (mState == State::Off) {
                                                                    }
                                                                    else {
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
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


