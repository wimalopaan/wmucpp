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

        struct RX;
        struct TX;

        struct SDA;
        struct SCL;

        struct MISO;
        struct MOSI;
        struct SCLK;
        struct CS;
        
        namespace detail {
            using Mcu::Components::Pin;
            using Mcu::Components::Timer;
            using Mcu::Components::Usart;
            using Mcu::Components::I2C;
            using Mcu::Components::SPI;
            
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

            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<A, 6>, Timer<3>, CC<1>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<A, 7>, Timer<3>, CC<2>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<B, 0>, Timer<3>, CC<3>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G4xx MCU>
            struct Impl<Pin<B, 7>, Timer<3>, CC<4>, MCU> : std::integral_constant<uint8_t, 10> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 11>, Timer<1>, CC<4>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 8>, Timer<1>, CC<1>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 3>, Timer<1>, CC<2>, MCU> : std::integral_constant<uint8_t, 1> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 0>, Timer<2>, CC<1>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 1>, Timer<2>, CC<2>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 2>, Timer<2>, CC<3>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 3>, Timer<2>, CC<4>, MCU> : std::integral_constant<uint8_t, 2> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 5>, Timer<2>, CC<1>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 15>, Timer<2>, CC<1>, MCU> : std::integral_constant<uint8_t, 2> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 3>, Timer<2>, CC<2>, MCU> : std::integral_constant<uint8_t, 2> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<C, 6>, Timer<3>, CC<1>, MCU> : std::integral_constant<uint8_t, 1> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 5>, Timer<3>, CC<2>, MCU> : std::integral_constant<uint8_t, 1> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 4>, Timer<3>, CC<1>, MCU> : std::integral_constant<uint8_t, 1> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 0>, Usart<4>, TX, MCU> : std::integral_constant<uint8_t, 4> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 1>, Usart<4>, RX, MCU> : std::integral_constant<uint8_t, 4> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 0>, Usart<102>, TX, MCU> : std::integral_constant<uint8_t, 4> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 2>, Usart<2>, TX, MCU> : std::integral_constant<uint8_t, 1> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<C, 6>, Usart<102>, TX, MCU> : std::integral_constant<uint8_t, 3> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<C, 7>, Usart<102>, RX, MCU> : std::integral_constant<uint8_t, 3> {};


            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 14>, Usart<2>, TX, MCU> : std::integral_constant<uint8_t, 1> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 15>, Usart<2>, RX, MCU> : std::integral_constant<uint8_t, 1> {};


            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<D, 2>, Usart<5>, RX, MCU> : std::integral_constant<uint8_t, 3> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<D, 3>, Usart<5>, TX, MCU> : std::integral_constant<uint8_t, 3> {};


            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 4>, Usart<6>, TX, MCU> : std::integral_constant<uint8_t, 3> {};

            // template<Mcu::Stm::G0xx MCU>
            // struct Impl<Pin<C, 6>, Usart<102>, TX, MCU> : std::integral_constant<uint8_t, 3> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 4>, Timer<14>, CC<1>, MCU> : std::integral_constant<uint8_t, 4> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 6>, Timer<3>, CC<1>, MCU> : std::integral_constant<uint8_t, 1> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 7>, Timer<3>, CC<2>, MCU> : std::integral_constant<uint8_t, 1> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 7>, Timer<17>, CC<1>, MCU> : std::integral_constant<uint8_t, 5> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 0>, Timer<3>, CC<3>, MCU> : std::integral_constant<uint8_t, 1> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 0>, Timer<1>, CC<2, Negativ>, MCU> : std::integral_constant<uint8_t, 2> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 6>, Timer<4>, CC<1>, MCU> : std::integral_constant<uint8_t, 9> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 7>, Timer<17>, CC<1, Negativ>, MCU> : std::integral_constant<uint8_t, 2> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 0>, Usart<5>, TX, MCU> : std::integral_constant<uint8_t, 8> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 8>, Usart<6>, TX, MCU> : std::integral_constant<uint8_t, 8> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 9>, Usart<6>, RX, MCU> : std::integral_constant<uint8_t, 8> {};


            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 2>, Usart<3>, TX, MCU> : std::integral_constant<uint8_t, 4> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 9>, Usart<3>, RX, MCU> : std::integral_constant<uint8_t, 4> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 6>, Usart<1>, TX, MCU> : std::integral_constant<uint8_t, 0> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 9>, Usart<1>, TX, MCU> : std::integral_constant<uint8_t, 1> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 10>, Usart<1>, RX, MCU> : std::integral_constant<uint8_t, 1> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 3>, Usart<101>, TX, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 10>, Usart<101>, RX, MCU> : std::integral_constant<uint8_t, 1> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 11>, Usart<101>, TX, MCU> : std::integral_constant<uint8_t, 1> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 6>, I2C<1>, SCL, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 7>, I2C<1>, SDA, MCU> : std::integral_constant<uint8_t, 6> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 13>, I2C<2>, SCL, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 14>, I2C<2>, SDA, MCU> : std::integral_constant<uint8_t, 6> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 3>, I2C<3>, SCL, MCU> : std::integral_constant<uint8_t, 6> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<B, 4>, I2C<3>, SDA, MCU> : std::integral_constant<uint8_t, 6> {};

            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 7>, SPI<1>, MOSI, MCU> : std::integral_constant<uint8_t, 0> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 6>, SPI<1>, MISO, MCU> : std::integral_constant<uint8_t, 0> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 1>, SPI<1>, SCLK, MCU> : std::integral_constant<uint8_t, 0> {};
            template<Mcu::Stm::G0xx MCU>
            struct Impl<Pin<A, 4>, SPI<1>, CS, MCU> : std::integral_constant<uint8_t, 0> {};
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
