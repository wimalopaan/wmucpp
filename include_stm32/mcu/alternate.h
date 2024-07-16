#pragma once

#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"
#include "components.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    namespace AlternateFunctions {
        struct Positiv;
        struct Negativ;
        template<uint8_t N, typename Pol = Positiv>
        struct CC : std::integral_constant<uint8_t, N> {};
        
        namespace detail {
            using Mcu::Components::Pin;
            using Mcu::Components::Timer;
            
            template<typename PinComponent, typename PeriComponent, typename Function, typename MCU>
            struct Impl;
    
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<A, 7>, Timer<1>, CC<1, Negativ>, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<A, 8>, Timer<1>, CC<1>, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<A, 9>, Timer<1>, CC<2>, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<A, 10>, Timer<1>, CC<3>, MCU> : std::integral_constant<uint8_t, 6> {};

            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<C, 13>, Timer<1>, CC<1, Negativ>, MCU> : std::integral_constant<uint8_t, 4> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<A, 12>, Timer<1>, CC<2, Negativ>, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<B, 15>, Timer<1>, CC<3, Negativ>, MCU> : std::integral_constant<uint8_t, 4> {};
            
        }
        
        template<typename Pin, typename Peripherie, typename Function, typename MCU = DefaultMcu>
        struct Mapper {
            using pin_component_t = Pin::component_t;
            using peri_component_t = Peripherie::component_t;
            static inline constexpr auto value = detail::Impl<pin_component_t, peri_component_t, Function, MCU>::value;
        };

        template<typename Pin, typename Peripherie, typename Function, typename MCU = DefaultMcu>
        static inline constexpr auto mapper_v = Mapper<Pin, Peripherie, Function, MCU>::value;        
    }
}
