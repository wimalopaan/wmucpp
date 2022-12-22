#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>
#include <etl/algorithm.h>

#include <std/vol_access>

#define ACCESS_ONCE(x) (*(volatile typeof(x) *)&(x))

template<typename T>
volatile T& volatile_access(T& v) {
    return static_cast<volatile T&>(v);
}

namespace Memory {
    struct Barrier final {
        inline Barrier() {
            _MemoryBarrier();
        }
        inline ~Barrier() {
            _MemoryBarrier();
        }
    };
    void barrier(const auto f) {
        Barrier b;
        f();
    }
}

namespace Atomic {
    struct DisableInterruptsRestore final {
        inline DisableInterruptsRestore() {
            cli(); // with barrier
        }
        inline ~DisableInterruptsRestore() {
            SREG = save;
            __asm__ volatile ("" ::: "memory");  // barrier added
        }
    private:
        uint8_t save{SREG};
    };
    
    void access(const auto f) {
        DisableInterruptsRestore di;
        Memory::barrier([&]{
            f();
        });
    }
}

volatile uint8_t mcuRegister;

struct SharedData {
    inline static uint32_t c;
    inline static uint8_t g;
    inline static bool flag;
};

//ISR(TCA0_CMP0_vect) {
//    Memory::barrier([]{
//        ++g;
//        if (g > 100) flag = true;
//    });
//}
//    Barrier();
//static void foo() __attribute__((__signal__, __used__));
void foo() {
//    _MemoryBarrier();
    
//        volatile uint8_t& rg = g;
//        ++rg;
    ++SharedData::c;
        ++SharedData::g;
        ++SharedData::g;
//    g = 2;
    if (SharedData::g > 100) SharedData::flag = true;
//    _MemoryBarrier();
}

int main() {
    while (true) {
        Memory::barrier([]{
            if (etl::nth_byte<1>(SharedData::c) == 0_B) {
                mcuRegister = 0x02;
            }
        });
        
        
////        Barrier(); // sync still needed
////        __sync_synchronize();
////            atomic_signal_fence(memory_order_acq_rel);
        
////        _MemoryBarrier(); // sync still needed
//        ++SharedData::g; // rmw
        
//        if (volatile_access(SharedData::flag)) {
//            volatile_access(SharedData::flag) = false;
//            mcuRegister = 0x01;            
//        }
        
////                    if (flag) {
////                        flag = false;
////                        mcuRegister = 0x01;            
////                    }

        
  
////        Atomic::access([]{
////            if (flag) {
////                flag = false;
////                mcuRegister = 0x01;            
////            }
////        });
    }
}
