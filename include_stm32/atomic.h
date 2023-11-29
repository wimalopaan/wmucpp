#pragma once 

#include "mcu_traits.h"
#include "concepts.h"

namespace Mcu::Arm {
    namespace Atomic {
        struct DisableInterruptsRestore {
            inline DisableInterruptsRestore() {
                __disable_irq();
            }
            inline ~DisableInterruptsRestore() {
                __set_PRIMASK(primask);
            }
        private:
            const uint32_t primask = __get_PRIMASK();
        };
        
        void access(const auto f) {
            DisableInterruptsRestore di; 
                f();
        }
    }
}


