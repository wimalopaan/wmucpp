#include <stm32g4xx.h>
#include <atomic>

namespace  {
    std::atomic_uint32_t a1;
    uint32_t a2;
    std::atomic_uint32_t a3;

//    volatile std::atomic_uint64_t l1;
//    volatile std::atomic_uint64_t l3;
    
    volatile uint64_t x1;
    volatile uint64_t x3;
    
    volatile float f1;
    volatile float f3;
}

int main(){
//    std::integral_constant<uint8_t, __FPU_PRESENT>::_;
    while(true) {
        x3 = x1; // LDRD not atomic
        
        f3 = f1;
        
        // 1
        uint32_t val;                                            
        do {                                                     
            val = __LDREXW(&a2);  
            val += 1;
        } while ((__STREXW(val, &a2)) != 0U); 
        
        // 2
        __disable_irq();
        std::atomic_signal_fence(std::memory_order_seq_cst);    // compile-time barrier really neccessary?
        ++a2;          
        std::atomic_signal_fence(std::memory_order_seq_cst);    // compile-time barrier really neccessary?
        __enable_irq();
        
        // 3
        std::atomic_fetch_add(&a1, 1);
        
        // 4
        std::atomic_signal_fence(std::memory_order_seq_cst);    // compile-time barrier
        std::atomic_fetch_add_explicit(&a1, 1, std::memory_order_relaxed);
        std::atomic_signal_fence(std::memory_order_seq_cst);

        asm("# xx");        
        // 5
        a1.load();
        a1.store(42);
    }
}

#pragma GCC push_options
#pragma GCC target("general-regs-only")
extern "C" {
void SysTick_Handler() __attribute__ ((isr));
void SysTick_Handler() {                              
    volatile uint32_t& va{a2};
    va += 1;
//    f1 *= 3.1415f;
    std::atomic_signal_fence(std::memory_order_seq_cst);    // compile-time barrier
    std::atomic_fetch_add_explicit(&a1, 1, std::memory_order_relaxed);
    std::atomic_signal_fence(std::memory_order_seq_cst);
}
}
#pragma GCC pop_options
