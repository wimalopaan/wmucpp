#pragma once

#include <cstdint>
#include "verticalcounter.h"
#include "etl/meta.h"

namespace External {
    namespace detail {
        namespace Button {
        }
    }
    
    template<typename PinList, typename Timer, typename DebounceCount> 
    struct ButtonControllerVertical;
    
    template<typename... Pins, typename Timer, typename C>
    struct ButtonControllerVertical<Meta::List<Pins...>, Timer, C> {
        static inline void init() {
            
        }
    private:
        static inline void update() {
        }
    };
}



