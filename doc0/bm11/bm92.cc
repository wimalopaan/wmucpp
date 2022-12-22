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
using ByteNumber = std::integral_constant<uint8_t, N>;

template<uint8_t N> 
using Counter = std::integral_constant<uint8_t, N>;

template<uint8_t N>
using NumberOfCounter = std::integral_constant<uint8_t, N>;

template<uint8_t N>
using Component = std::integral_constant<uint8_t, N>;

template<typename ComponentNumber, typename NumberOfCounters>
struct Timer {
    static void isr() {
        for(auto& c : cc) ++c;
    }
    template<typename Byte, typename Counter>
    static void whenByteIsZero(auto f) {
        Memory::barrier([&]{ // simple barrier, no cli/sei
            if (etl::nth_byte<Byte::value>(cc[Counter::value]) == 0_B) {
                f();
            }
        });
    }
private:
    inline static std::array<uint32_t, NumberOfCounters::value> cc;
};

using timer0 = Timer<Component<0>, NumberOfCounter<4>>;

ISR(TCA0_CMP0_vect) {
    timer0::isr();
}

int main() {
    while (true) {
        timer0::whenByteIsZero<ByteNumber<1>, Counter<0>>([]{
            VPORTA_OUT = 0x01;
        });
    }
}
