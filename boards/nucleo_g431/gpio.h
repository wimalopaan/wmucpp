#pragma once

namespace Mcu::Stm {
    using namespace Units::literals;

    template<typename Letter, typename MCU>
    struct GPIO {
        static inline /*constexpr */ GPIO_TypeDef* const mcuGpio = reinterpret_cast<GPIO_TypeDef*>(Mcu::Stm::Address<GPIO<Letter>>::value);
        static inline void init() {
            RCC->AHB2ENR |= Letter::ahb2Bit;
        }
    };
    
    template<typename GP, uint8_t N, typename MCU = void>
    struct Pin {
        static inline /*constexpr */ GPIO_TypeDef* const& mcuGpio = GP::mcuGpio;
        static inline constexpr uint8_t moderPos   =  (2 * N);
        static inline constexpr uint32_t moderMask =  0x03UL << moderPos;
        
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
        static inline void set() {
            mcuGpio->BSRR |= (0x01UL << N);
        }
        static inline void reset() {
            mcuGpio->BSRR |= (0x01UL << (N + 16));
        }
        static inline void toggle() {
            mcuGpio->BSRR = (mcuGpio->ODR ^ (0x01UL << N)) | (0x01UL << (N + 16));
        }
    };
    
}
