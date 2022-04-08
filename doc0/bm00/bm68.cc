#include <mcu/avr.h>
#include <etl/fsm.h>

volatile uint8_t r = 0;
volatile uint8_t x = 0;

namespace {
    
    enum class StateE : uint8_t {Off, Start, Running, Test0, Test1, Test2, Test3, Test4, Test5, Test6, Test7, Test8, Test9, Error};
    
    template<StateE S> struct State;

    using Off     = State<StateE::Off>;
    using Start   = State<StateE::Start>;
    using Running = State<StateE::Running>;
    using Error   = State<StateE::Error>;
    using Test0   = State<StateE::Test0>;
    using Test1   = State<StateE::Test1>;
    using Test2   = State<StateE::Test2>;
    using Test3   = State<StateE::Test3>;
    using Test4   = State<StateE::Test4>;
    using Test5   = State<StateE::Test5>;
    using Test6   = State<StateE::Test6>;
    using Test7   = State<StateE::Test7>;
    using Test8   = State<StateE::Test8>;
    using Test9   = State<StateE::Test9>;
    
    template<>
    struct State<StateE::Off> {
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
    template<>
    struct State<StateE::Start> {
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
    template<>
    struct State<StateE::Test0> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 10) {
                S::template toState<Test1>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 40;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test1> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 11) {
                S::template toState<Test2>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 41;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test2> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 12) {
               S::template toState<Test3>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 42;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test3> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 13) {
                S::template toState<Test4>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 43;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test4> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 14) {
                S::template toState<Test5>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 44;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test5> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 15) {
                S::template toState<Test6>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 45;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test6> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 16) {
                S::template toState<Test7>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 46;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test7> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 17) {
                S::template toState<Test8>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 47;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test8> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 18) {
                S::template toState<Test9>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 48;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Test9> {
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 19) {
                S::template toState<Running>();
            }
        }            
        static inline constexpr void onEnter() {
            x = 49;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<>
    struct State<StateE::Running> {
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
    template<>
    struct State<StateE::Error> {
        template<typename S>
        static inline constexpr void process(const uint8_t) {
        }            
        static inline constexpr void onEnter() {
        }            
        static inline constexpr void onExit() {
        }            
    };
    
    
    using sp = FSM::Enum::StateProcessor<StateE, Off, Meta::List<Off, Start, Running,
                                                           Test0, Test1, Test2, Test3, Test4, Test5, Test6, Test7, Test8, Test9,
                                                           Error>, uint8_t>;
}

int main() {
    while(true) {
        const uint8_t b = r;
        sp::process(b);
    }
}
