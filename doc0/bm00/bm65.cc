#include <mcu/avr.h>
#include <etl/fsm.h>

volatile uint8_t r = 0;
volatile uint8_t x = 0;

namespace {
    struct Off;
    struct Start;
    struct Running;
    struct Test0;
    struct Test1;
    struct Test2;
    struct Test3;
    struct Test4;
    struct Test5;
    struct Test6;
    struct Test7;
    struct Test8;
    struct Test9;
    struct Error;
    
    struct Off {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 1) {
                S::template toState<Start>();                    
            }
        }            
        static inline constexpr void onEnter() {
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Start {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
                S::template toState<Test0>();                    
            }
            else if (b == 0) {
                S::template toState<Error>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 43;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test0 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 0) {
                S::template toState<Test1>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 40;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test1 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 1) {
                S::template toState<Test2>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 41;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test2 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 2) {
                S::template toState<Test3>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 42;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test3 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
                S::template toState<Test4>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 43;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test4 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 4) {
                S::template toState<Test5>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 44;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test5 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 5) {
                S::template toState<Test6>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 45;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test6 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 6) {
                S::template toState<Test7>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 46;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test7 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 7) {
                S::template toState<Test8>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 47;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test8 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 8) {
                S::template toState<Test9>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 48;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test9 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 9) {
                S::template toState<Running>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 49;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Running {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
                S::template toState<Off>();                    
            }
            else if (b > 100) {
                S::template toState<Error>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 44;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Error {
        template<typename S>
        static inline constexpr void process(const uint8_t) {
        }            
        static inline constexpr void onEnter() {
        }            
        static inline constexpr void onExit() {
        }            
    };
    
    using sp = FSM::Simple::StateProcessor<Off, Meta::List<Off, Running, Start, Error,
                                                           Test0, Test1, Test2, Test3, Test4, Test5, Test6, Test7, Test8, Test9>, uint8_t>;
}

namespace X {
    struct Off;
    struct Start;
    struct Running;
    struct Test0;
    struct Test1;
    struct Test2;
    struct Test3;
    struct Test4;
    struct Test5;
    struct Test6;
    struct Test7;
    struct Test8;
    struct Test9;
    struct Error;
    
    struct Off {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 1) {
//                S::template toState<Start>();                    
                S::toState(S::template index<Start>());
            }
        }            
        static inline constexpr void onEnter() {
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Start {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
//                S::template toState<Test0>();                    
                S::toState(S::template index<Test0>());
            }
            else if (b == 0) {
//                S::template toState<Error>();                    
                S::toState(S::template index<Error>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 43;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test0 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 0) {
//                S::template toState<Test1>();                    
                S::toState(S::template index<Test1>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 40;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test1 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 1) {
//                S::template toState<Test2>();                    
                S::toState(S::template index<Test2>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 41;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test2 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 2) {
//                S::template toState<Test3>();                    
                S::toState(S::template index<Test3>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 42;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test3 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
//                S::template toState<Test4>();                    
                S::toState(S::template index<Test4>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 43;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test4 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 4) {
//                S::template toState<Test5>();                    
                S::toState(S::template index<Test5>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 44;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test5 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 5) {
//                S::template toState<Test6>();                    
                S::toState(S::template index<Test6>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 45;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test6 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 6) {
//                S::template toState<Test7>();                    
                S::toState(S::template index<Test7>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 46;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test7 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 7) {
//                S::template toState<Test8>();                    
                S::toState(S::template index<Test8>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 47;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test8 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 8) {
//                S::template toState<Test9>();                    
                S::toState(S::template index<Test9>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 48;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Test9 {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 9) {
//                S::template toState<Running>();                    
                S::toState(S::template index<Running>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 49;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Running {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
//                S::template toState<Off>();                    
                S::toState(S::template index<Off>());
            }
            else if (b > 100) {
//                S::template toState<Error>();                    
                S::toState(S::template index<Error>());
            }
        }            
        static inline constexpr void onEnter() {
            x = 44;
        }            
        static inline constexpr void onExit() {
        }            
    };
    struct Error {
        template<typename S>
        static inline constexpr void process(const uint8_t) {
        }            
        static inline constexpr void onEnter() {
        }            
        static inline constexpr void onExit() {
        }            
    };
    
    using sp = FSM::Simple::StateProcessor<Off, Meta::List<Off, Running, Start, Error,
                                                           Test0, Test1, Test2, Test3, Test4, Test5, Test6, Test7, Test8, Test9>, uint8_t>;
}

int main() {
    while(true) {
        const uint8_t b = r;
        sp::process(b);
    }
}

