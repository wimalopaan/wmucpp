#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>
#include <etl/algorithm.h>

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
//        Memory::barrier([&]{ // not needed here because `di` uses already barrier
            f();
//        });
    }
}

template<uint8_t N>
struct Timer {
    static void isr() {
        ++c;
    }
    template<uint8_t B>
    static void whenByteIsZero(auto f) {
        Memory::barrier([&]{ // simple barrier, no cli/sei
            if (etl::nth_byte<B>(c) == 0_B) {
                f();
            }
        });
    }
private:
    inline static uint32_t c;
};

using timer0 = Timer<0>;

ISR(TCA0_CMP0_vect) {
    timer0::isr();
}

int main() {
    while (true) {
        timer0::whenByteIsZero<1>([]{
            VPORTA_OUT = 0x01;
        });
    }
}
