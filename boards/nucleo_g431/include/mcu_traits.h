#pragma once

namespace Mcu {
    template<bool F>
    using UseInterrupts = std::integral_constant<bool, F>;
    
    struct Output;
    struct Input;
    
    struct High;
    
    namespace Stm {
        template<typename MCU> struct isG4xx : std::false_type {};
        template<> struct isG4xx<Stm32G431> : std::true_type {};
            
        template<typename MCU> concept G4xx = isG4xx<MCU>::value;
        
        struct A {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 0;
        };    
        struct B {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 1;
        };
        struct C {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 2;
        };
        struct F {
            static inline constexpr uint8_t ahb2Bit = 0x01 << 5;
        };
        
        template<typename C> struct Address;

    }
}
