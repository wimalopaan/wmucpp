#include <cstdint>
#include <mcu/avr.h>

#include <etl/algorithm.h>

volatile uint8_t r;

//namespace {
//    void periodic() {
//        auto part1 = []{ r = 1;};
//        auto part2 = []{ r = 2;};
//        auto part3 = []{};
//        etl::circular_call(part1, part2, part3);
//    }        
    
//    void co_periodic() {
//        r = 1;
////        yield();
//        r = 2;
////        yield();
//    }
//}


namespace  {
    struct A;

    template<auto F1, auto... FF>
    struct Coroutine {
        static constexpr uint8_t size = 1 + sizeof...(FF);
        
        static inline void process() {
//            std::integral_constant<uint8_t, size>::_;
            etl::select_f(mPart.toInt(), F1, FF...);
            ++mPart;
        }
    private:
        static inline etl::uint_ranged_circular<uint8_t, 0, size - 1> mPart;
    };
    
    template<typename P1>
    struct FSM {
        
        static constexpr auto u1 = []{ r = 1; }; 
        static constexpr auto u2 = []{ r = 2; }; 

        static constexpr auto u10 = []{ r = 10; }; 
        static constexpr auto u20 = []{ r = 20; }; 
        
        using coro1 = Coroutine<u1, u2, u10, u20>;
//        using coro1 = Coroutine<u1, u2>;
//        using coro2 = Coroutine<u10, u20>;
        
        static void periodic() {
            coro1::process();    
//            coro2::process();    
        }        
    };
}

using fsm = FSM<A>;

int main() {
    while(true) { 
        fsm::periodic();
    }
}
