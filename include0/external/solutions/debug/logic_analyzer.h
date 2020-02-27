#pragma once

#include <mcu/internals/port.h>

namespace External::Debug {
    
    template<AVR::Concepts::Pin Trigger, typename PG>
    struct TriggeredParallel {
        using pin_list = typename PG::pin_list;
        static_assert(!Meta::contains_v<pin_list, Trigger>, "Trigger Pin must be different");
        
        inline static constexpr void init() {
            PG::template dir<AVR::Output>();
            Trigger::template dir<AVR::Output>();
        }        
        template<auto V>
        inline static constexpr void set() {
            Trigger::off();
            PG::template set<V>();
            Trigger::on();
        }
    };
}
