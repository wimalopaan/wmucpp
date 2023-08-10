#include <cstdint>
#include "mcu.h"
#include "isr.h"

namespace  {
    struct SystemTimer final {
        SystemTimer() = delete;
        static void isr() {
            ++counter;
        }
    private:
        static inline Mcu::Arm::Interrupt::volatile_atomic<uint32_t> counter;
    public:
        // Policy: außerhalb der isr() nur die volatile& verwenden: damit volatile-access und(!) Atomarität
        static inline volatile const auto& vCounter{counter};
    };
    
    volatile uint32_t r1;
    volatile uint32_t r2;
}

int main(){
    while(true) {
        SystemTimer::vCounter.use([](const auto& v){
            r1 = v;
        });
        
        SystemTimer::vCounter.on(1000, Mcu::Arm::Interrupt::non_atomic{}, [](const auto& v){
            r2 = v;
        });        
    }
}

#pragma GCC push_options
#pragma GCC target("general-regs-only")
extern "C" {
void SysTick_Handler() __attribute__ ((isr));
void SysTick_Handler() {            
    SystemTimer::isr();
}
}
#pragma GCC pop_options
