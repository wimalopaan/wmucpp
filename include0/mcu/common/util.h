#pragma once

#include <cstdint>

#include "groups.h"

namespace AVR {
    using namespace External::Units;
    
    namespace Util {
        
        template<typename... CC>
        struct StaticInitializer {
            StaticInitializer() {
                (CC::init(), ...);
            }
        };

        template<typename... CC>
        struct Periodic {
            static inline void periodic() {
                (CC::periodic(), ...);
            }
        };
        
    }
}
