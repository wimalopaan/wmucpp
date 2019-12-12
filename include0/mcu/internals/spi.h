#pragma once

#include <cstdint>
#include <etl/types.h>
#include <etl/rational.h>
#include <etl/type_traits.h>

#include "../common/concepts.h"
#include "../common/isr.h"

#include "portmux.h"

namespace AVR {
    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::NamedFlag useISR = etl::NamedFlag<false>, typename MCU = DefaultMcuType> struct Spi;
    
    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::NamedFlag useISR, AVR::Concepts::At01Series MCU>
    struct Spi<CP, useISR, MCU> final {
        static inline constexpr auto N = CP::component_type::value;

        static constexpr auto mcu_spi = getBaseAddr<typename MCU::Spi, N>;
        static_assert(N < AVR::Component::Count<typename MCU::Spi>::value, "wrong number of spi");
        
        inline static void init() {
            
        }
        
        
        
        
    };
}

