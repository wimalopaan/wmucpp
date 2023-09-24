#include "mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu_traits.h"
#include "components.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    template<uint8_t N>
    struct CompareChannel {};

    namespace detail {
//        using namespace Mcu::Components;
        
        template<typename PinComponent, typename PeriComponent, typename Function, typename MCU>
        struct AlternateMapperImpl;
        
//        template<> struct AlternateMapperImpl<Pin<B, 4>, Timer<3>, CompareChannel<1>, Stm32G431> : std::integral_constant<uint8_t, 2> {};
//        template<> struct AlternateMapperImpl<Pin<B, 5>, Timer<3>, CompareChannel<2>, Stm32G431> : std::integral_constant<uint8_t, 2> {};
//        template<> struct AlternateMapperImpl<Pin<B, 0>, Timer<3>, CompareChannel<3>, Stm32G431> : std::integral_constant<uint8_t, 2> {};
//        template<> struct AlternateMapperImpl<Pin<B, 1>, Timer<3>, CompareChannel<4>, Stm32G431> : std::integral_constant<uint8_t, 2> {};
    }
    
    
    template<typename Pin, typename Peripherie, typename Function, typename MCU = DefaultMcu>
    struct AlternateMapper {
        using pin_component_t = Pin::component_t;
        using peri_component_t = Peripherie::component_t;
        
        static inline constexpr auto value = detail::AlternateMapperImpl<pin_component_t, peri_component_t, Function, MCU>::value;
    };
}
