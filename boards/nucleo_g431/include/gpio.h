#pragma once

#include "mcu.h"
#include "mcu_traits.h"
#include "concepts.h"
#include "components.h"

#include <type_traits>
#include <concepts>

namespace Mcu::Stm {
    using namespace Units::literals;

    
#ifdef USE_MCU_STM_V2
    inline 
#endif
    namespace V2 {
        
    }
    
#ifdef USE_MCU_STM_V1
    inline 
#endif
    namespace V1 {
        
    }
    
    template<typename Letter, typename MCU = void>
    struct GPIO;

    template<typename Letter, typename MCU>
    struct GPIO {
        using port_t = Letter;
        using mcu_t = MCU;
        static inline /*constexpr */ GPIO_TypeDef* const mcuGpio = reinterpret_cast<GPIO_TypeDef*>(Mcu::Stm::Address<GPIO<Letter, MCU>>::value);
        static inline void init() {
            RCC->AHB2ENR |= Letter::ahb2Bit;
        }
    };
    
    template<G4xx MCU> 
    struct Address<GPIO<A, MCU>> {
        static inline constexpr uintptr_t value = GPIOA_BASE;
    };
    template<G4xx MCU> 
    struct Address<GPIO<B, MCU>> {
        static inline constexpr uintptr_t value = GPIOB_BASE;
    };
    template<G4xx MCU> 
    struct Address<GPIO<F, MCU>> {
        static inline constexpr uintptr_t value = GPIOF_BASE;
    };
    
    template<typename GP, uint8_t N, typename MCU = void>
    struct Pin {
//        using component_t = Mcu::Components::Pin<typename GP::port_t, N>;
        
        using gpio_t = GP;
        using number_t = std::integral_constant<uint8_t, N>;
        
        static inline /*constexpr */ GPIO_TypeDef* const& mcuGpio = GP::mcuGpio;
        static inline constexpr uint8_t moderPos   =  (2 * N);
        static inline constexpr uint32_t moderMask =  0x03UL << moderPos;
    
        static inline void analog() {
            mcuGpio->MODER |= GPIO_MODER_MODE0 << moderPos;
        }

        template<bool ON = true>
        static inline void pullup() {
            if constexpr(ON) {
                MODIFY_REG(mcuGpio->PUPDR, (GPIO_PUPDR_PUPD0 << moderPos), (0b01 << moderPos));;
            }
            else {
                MODIFY_REG(mcuGpio->PUPDR, (GPIO_PUPDR_PUPD0 << moderPos), (0b00 << moderPos));;
            }
        }
        template<bool ON = true>
        static inline void pulldown() {
            if constexpr(ON) {
                MODIFY_REG(mcuGpio->PUPDR, (GPIO_PUPDR_PUPD0 << moderPos), (0b10 << moderPos));;
            }
            else {
                MODIFY_REG(mcuGpio->PUPDR, (GPIO_PUPDR_PUPD0 << moderPos), (0b00 << moderPos));;
            }
        }
        
        template<typename S = Mcu::High>
        static inline void speed() {
            mcuGpio->OSPEEDR |= (0x11 << moderPos);
        }
        template<typename D>
        static inline void dir() {
            auto temp = mcuGpio->MODER;
            temp &= ~(GPIO_MODER_MODE0 << moderPos);
            if constexpr(std::is_same_v<D, Mcu::Output>) {
                temp |= 0x01UL << moderPos;
            }
            mcuGpio->MODER = temp;
        }
        static inline void set(const bool b) {
            if (b) {
                set();
            }
            else {
                reset();
            }
        }
        static inline void set() {
            mcuGpio->BSRR |= (0x01UL << N);
        }
        static inline void reset() {
            mcuGpio->BSRR |= (0x01UL << (N + 16));
        }
        static inline void toggle() {
            mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
        }
        static inline void openDrain() {
            mcuGpio->OTYPER |= (0x01 << N);
        }
        static inline void afunction(const uint8_t f) {
            auto temp = mcuGpio->MODER;
            temp &= ~(GPIO_MODER_MODE0 << moderPos);
            temp |= (0x02 << moderPos);
            mcuGpio->MODER = temp;
            
            if constexpr(N < 8) {
                uint32_t temp = mcuGpio->AFR[0];
                temp &= ~(0x0f << (N * 4));
                temp |= (f & 0x0f) << (N * 4);
                mcuGpio->AFR[0] = temp;
            }
            else {
                uint32_t temp = mcuGpio->AFR[1];
                temp &= ~(0x0f << ((N - 8) * 4));
                temp |= (f & 0x0f) << ((N - 8) * 4);
                mcuGpio->AFR[1] = temp;
            }
        }
    };
    
}
