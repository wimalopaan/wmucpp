#pragma once

#include "concepts.h"

namespace AVR {
    
    namespace ADC {
        template<typename NumberType, typename MMCU = DefaultMcuType>
        struct ChannelPinMapper;

        template<auto V, AVR::Concepts::AtDb64 MMCU>
        requires(V < 8)
        struct ChannelPinMapper<std::integral_constant<decltype(V), V>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::D>, V>;  
        };
        template<auto V, AVR::Concepts::AtDb64 MMCU>
        requires((V >= 8) && (V < 16))
        struct ChannelPinMapper<std::integral_constant<decltype(V), V>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::E>, V - 8>;  
        };
        template<auto V, AVR::Concepts::AtDb64 MMCU>
        requires((V >= 16) && (V <= 21))
        struct ChannelPinMapper<std::integral_constant<decltype(V), V>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::F>, V - 16>;  
        };
        template<AVR::Concepts::AtDb64 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(0x42), 0x42>, MMCU> {
            using pin_type = void;
        };
        
        
        template<auto V, AVR::Concepts::AtDa32 MMCU>
        requires(V < 8)
        struct ChannelPinMapper<std::integral_constant<decltype(V), V>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::D>, V>;  
        };
        template<auto V, AVR::Concepts::AtDa32 MMCU>
        requires((V >= 16) && (V <= 21))
        struct ChannelPinMapper<std::integral_constant<decltype(V), V>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::F>, V - 16>;  
        };
        template<AVR::Concepts::AtDa32 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(0x42), 0x42>, MMCU> {
            using pin_type = void;
        };
        
        template<auto V, AVR::Concepts::AtMega0 MMCU>
        requires(V < 8)
        struct ChannelPinMapper<std::integral_constant<decltype(V), V>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::D>, V>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(8), 8>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::E>, 0>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(9), 9>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::E>, 1>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(10), 10>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::E>, 2>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(11), 11>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::E>, 3>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(12), 12>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::F>, 2>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(13), 13>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::F>, 3>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(14), 14>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::F>, 4>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(15), 15>, MMCU> {
            using pin_type = AVR::Pin<AVR::Port<AVR::F>, 5>;  
        };
        template<AVR::Concepts::AtMega0 MMCU>
        struct ChannelPinMapper<std::integral_constant<decltype(0x1e), 0x1e>, MMCU> {
            using pin_type = void;
        };

        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(0), 0>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 0>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(1), 1>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 1>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(2), 2>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 2>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(3), 3>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 3>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(4), 4>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 4>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(5), 5>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 5>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(6), 6>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 6>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(7), 7>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 7>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(10), 10>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::B>, 1>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(11), 11>, AVR::ATTiny1614> {
            using pin_type = AVR::Pin<AVR::Port<AVR::B>, 0>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(0x1d), 0x1d>, AVR::ATTiny1614> {
            using pin_type = void;
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(0x1e), 0x1e>, AVR::ATTiny1614> {
            using pin_type = void;
        };

        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(1), 1>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 1>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(2), 2>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 2>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(3), 3>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 3>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(4), 4>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 4>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(5), 5>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 5>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(6), 6>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 6>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(7), 7>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 7>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(10), 10>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::B>, 1>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(11), 11>, AVR::ATTiny1624> {
            using pin_type = AVR::Pin<AVR::Port<AVR::B>, 0>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(0x1d), 0x1d>, AVR::ATTiny1624> {
            using pin_type = void;
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(0x1e), 0x1e>, AVR::ATTiny1624> {
            using pin_type = void;
        };


        
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(0), 0>, AVR::ATTiny412> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 0>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(1), 1>, AVR::ATTiny412> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 1>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(2), 2>, AVR::ATTiny412> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 2>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(3), 3>, AVR::ATTiny412> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 3>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(6), 6>, AVR::ATTiny412> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 6>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(7), 7>, AVR::ATTiny412> {
            using pin_type = AVR::Pin<AVR::Port<AVR::A>, 7>;  
        };
        template<>
        struct ChannelPinMapper<std::integral_constant<decltype(0x1e), 0x1e>, AVR::ATTiny412> {
            using pin_type = void;
        };
        
    }
    
    namespace Portmux {
        struct Default {};
        struct Alt1 {};
        struct Alt2 {};
        struct AltA {};
        struct AltB {};
        struct AltC {};
        struct AltD {};
        struct AltE {};
        struct AltF {};
        struct AltG {};
        struct None {};
        
        
        template<typename Component, typename Place>
        struct Position {
            using component_type = Component;
            using place_type = Place;
        };
        
        template<typename CompPos, typename MCU = DefaultMcuType>
        struct Map;

        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Spi<0>, Default>, MCU> {
            using sspin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
            using mosipin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using misopin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
            using sckpin = AVR::Pin<AVR::Port<AVR::A>, 6>; 
        };
        
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Spi<0>, Default>, MCU> {
            using sspin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
            using mosipin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using misopin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
            using sckpin = AVR::Pin<AVR::Port<AVR::A>, 6>; 
        };

        template<>
        struct Map<Position<AVR::Component::Spi<0>, Default>, AVR::ATMega4809> {
            using sspin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
            using mosipin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using misopin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
            using sckpin = AVR::Pin<AVR::Port<AVR::A>, 6>; 
        };
        template<>
        struct Map<Position<AVR::Component::Spi<0>, Alt1>, AVR::ATMega4809> {
            using sspin = AVR::Pin<AVR::Port<AVR::C>, 3>; 
            using mosipin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using misopin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
            using sckpin = AVR::Pin<AVR::Port<AVR::C>, 2>; 
        };
        
        template<>
        struct Map<Position<AVR::Component::Spi<0>, Default>, AVR::ATTiny1614> {
            using sspin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using mosipin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using misopin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
            using sckpin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
        };

        template<>
        struct Map<Position<AVR::Component::Spi<0>, Default>, AVR::ATTiny412> {
            using sspin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using mosipin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using misopin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
            using sckpin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
        };
        template<>
        struct Map<Position<AVR::Component::Spi<0>, Alt1>, AVR::ATTiny412> {
            using sspin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using mosipin = AVR::Pin<AVR::Port<AVR::A>, 6>; 
            using misopin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
            using sckpin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
        };
        
        template<>
        struct Map<Position<AVR::Component::Usart<0>, Default>, AVR::ATTiny412> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 6>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
        };
        template<>
        struct Map<Position<AVR::Component::Usart<0>, Alt1>, AVR::ATTiny412> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
        };

        template<>
        struct Map<Position<AVR::Component::Usart<0>, Default>, AVR::ATTiny1614> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 3>; 
        };
        template<>
        struct Map<Position<AVR::Component::Usart<0>, Alt1>, AVR::ATTiny1614> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
        };

        template<>
        struct Map<Position<AVR::Component::Usart<0>, Default>, AVR::ATTiny1624> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 3>; 
        };
        template<>
        struct Map<Position<AVR::Component::Usart<0>, Alt1>, AVR::ATTiny1624> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
        };
        
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
        };
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
        };
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 1>; 
        };
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };


        
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 1>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<3>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<3>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 5>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<4>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::E>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::E>, 1>; 
        };
        template<AVR::Concepts::AtDa48 MCU>
        struct Map<Position<AVR::Component::Usart<4>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::E>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::E>, 5>; 
        };
        
        
        
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 1>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<3>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<3>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 5>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<4>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::E>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::E>, 1>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<4>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::E>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::E>, 5>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<5>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::G>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::G>, 1>; 
        };
        template<AVR::Concepts::AtDb64 MCU>
        struct Map<Position<AVR::Component::Usart<5>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::G>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::G>, 5>; 
        };
        
        
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<0>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<1>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 1>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<2>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<3>, Default>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Usart<3>, Alt1>, MCU> {
            using txpin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
            using rxpin = AVR::Pin<AVR::Port<AVR::B>, 5>; 
        };

        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Tca<0>, Default>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        
        template<>
        struct Map<Position<AVR::Component::Tca<0>, Default>, AVR::ATMega4809> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<>
        struct Map<Position<AVR::Component::Tca<0>, Default>, AVR::ATTiny1614> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<>
        struct Map<Position<AVR::Component::Tca<0>, Alt1>, AVR::ATTiny1614> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::B>, 3>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };

        template<>
        struct Map<Position<AVR::Component::Tca<0>, Default>, AVR::ATTiny1624> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<>
        struct Map<Position<AVR::Component::Tca<0>, Alt1>, AVR::ATTiny1624> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::B>, 3>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        
        template<>
        struct Map<Position<AVR::Component::Tca<0>, Default>, AVR::ATTiny412> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
        };
        template<>
        struct Map<Position<AVR::Component::Tca<0>, Alt1>, AVR::ATTiny412> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
        };

        template<>
        struct Map<Position<AVR::Component::Tcd<0>, Default>, AVR::ATTiny412> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 6>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 7>; 
        };

        template<AVR::Concepts::AtDxSmall MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltA>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<AVR::Concepts::AtDxSmall MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltC>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::C>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::C>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtDxSmall MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltD>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::D>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::D>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::D>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::D>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::D>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::D>, 5>; 
        };

        template<AVR::Concepts::AtDxSmall MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltF>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::F>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::F>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::F>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::F>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };
        
        
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltA>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltB>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::B>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::B>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltC>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::C>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::C>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltD>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::D>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::D>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::D>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::D>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::D>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::D>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltE>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::E>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::E>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::E>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::E>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::E>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::E>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltF>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::F>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::F>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::F>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::F>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltG>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::G>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::G>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::G>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::G>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::G>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::G>, 5>; 
        };
        
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<1>, AltB>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::B>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::B>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<1>, AltC>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::C>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::C>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<1>, AltE>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::E>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::E>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::E>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::E>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::E>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::E>, 5>; 
        };
        template<AVR::Concepts::AtDxLarge MCU>
        struct Map<Position<AVR::Component::Tca<1>, AltG>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::G>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::G>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::G>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::G>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::G>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::G>, 5>; 
        };
        
        
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Tcb<0>, Default>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
        };
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Tcb<1>, Default>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
        };        
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Tcb<2>, Default>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
        };        

        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Tcb<0>, Alt1>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
        };
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Tcb<1>, Alt1>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };        
        template<AVR::Concepts::AtDa32 MCU>
        struct Map<Position<AVR::Component::Tcb<2>, Alt1>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
        };        

        
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltA>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::A>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::A>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::A>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltB>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::B>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::B>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::B>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::B>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::B>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltC>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::C>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::C>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::C>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::C>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltD>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::D>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::D>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::D>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::D>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::D>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::D>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltE>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::E>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::E>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::E>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::E>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::E>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::E>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tca<0>, AltF>, MCU> {
            using wo0pin = AVR::Pin<AVR::Port<AVR::F>, 0>; 
            using wo1pin = AVR::Pin<AVR::Port<AVR::F>, 1>; 
            using wo2pin = AVR::Pin<AVR::Port<AVR::F>, 2>; 
            using wo3pin = AVR::Pin<AVR::Port<AVR::F>, 3>; 
            using wo4pin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
            using wo5pin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<0>, Default>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 2>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<0>, Alt1>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::F>, 4>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<1>, Default>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<1>, Alt1>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::F>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<2>, Default>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::C>, 0>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<2>, Alt1>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::B>, 4>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<3>, Default>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::B>, 5>; 
        };
        template<AVR::Concepts::AtMega0 MCU>
        struct Map<Position<AVR::Component::Tcb<3>, Alt1>, MCU> {
            using wopin = AVR::Pin<AVR::Port<AVR::C>, 1>; 
        };

        template<>
        struct Map<Position<AVR::Component::Tcb<0>, Default>, AVR::ATTiny1614> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<>
        struct Map<Position<AVR::Component::Tcb<1>, Default>, AVR::ATTiny1614> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
        };

        template<>
        struct Map<Position<AVR::Component::Tcb<0>, Default>, AVR::ATTiny1624> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 5>; 
        };
        template<>
        struct Map<Position<AVR::Component::Tcb<1>, Default>, AVR::ATTiny1624> {
            using wopin = AVR::Pin<AVR::Port<AVR::A>, 3>; 
        };
    }
    
    namespace Util::SoftUart {
        
        template<uint8_t N, typename MCU = DefaultMcuType>
        struct Icp;
        
        template<>
        struct Icp<1, AVR::ATMega1284P> {
            Icp() = delete;
            using PortD = AVR::Port<AVR::D>;
            using icp = AVR::Pin<PortD, 6>; // icp1
        };
        template<>
        struct Icp<3, AVR::ATMega1284P> {
            Icp() = delete;
            using PortB = AVR::Port<AVR::B>;
            using icp = AVR::Pin<PortB, 5>; // icp3      
        };
        
        template<>
        struct Icp<1, AVR::ATMega328P> {
            Icp() = delete;
            using PortB = AVR::Port<AVR::B>;
            using icp = AVR::Pin<PortB, 0>; // icp1
        };
        
        template<>
        struct Icp<1, AVR::ATMega328PB> {
            Icp() = delete;
            using PortB = AVR::Port<AVR::B>;
            using icp = AVR::Pin<PortB, 0>; // icp1
        };
        
        template<>
        struct Icp<3, AVR::ATMega328PB> {
            Icp() = delete;
            using PortE = AVR::Port<AVR::E>;
            using icp = AVR::Pin<PortE, 2>; // icp3
        };
        
        template<>
        struct Icp<0, AVR::ATTiny84> {
            Icp() = delete;
            using PortA = AVR::Port<AVR::A>;
            using icp = AVR::Pin<PortA, 7>; // icp
        };
        
        template<uint8_t N, typename MCU = DefaultMcuType>
        struct ExternalInterrupt;
        
        template<typename MCU>
        struct ExternalInterrupt<0, MCU> {
            ExternalInterrupt() = delete;
            using PortD = AVR::Port<AVR::D>;
            using pin = AVR::Pin<PortD, 2>; // int 0
            inline static constexpr auto edge_falling = MCU::Interrupt::EIControl::isc01;
            inline static constexpr auto mask = MCU::Interrupt::EIMask::int0;
            inline static constexpr auto flag = MCU::Interrupt::EIFlags::int0;
        };
        template<typename MCU>
        struct ExternalInterrupt<1, MCU> {
            ExternalInterrupt() = delete;
            using PortD = AVR::Port<AVR::D>;
            using pin = AVR::Pin<PortD, 3>; // int 1
            inline static constexpr auto edge_falling = MCU::Interrupt::EIControl::isc11;
            inline static constexpr auto mask = MCU::Interrupt::EIMask::int1;
            inline static constexpr auto flag = MCU::Interrupt::EIFlags::int1;
        };
        template<typename MCU>
        struct ExternalInterrupt<2, MCU> {
            ExternalInterrupt() = delete;
            using PortB = AVR::Port<AVR::B>;
            using pin = AVR::Pin<PortB, 2>; // int 2
            inline static constexpr auto edge_falling = MCU::Interrupt::EIControl::isc21;
            inline static constexpr auto mask = MCU::Interrupt::EIMask::int2;
            inline static constexpr auto flag = MCU::Interrupt::EIFlags::int2;
        };
        
        template<>
        struct ExternalInterrupt<0, AVR::ATTiny85> {
            ExternalInterrupt() = delete;
            using PortB = AVR::Port<AVR::B>;
            using pin = AVR::Pin<PortB, 2>; // int 0
        };
        
    }
    
    namespace Util::Timer {
        using megahertz = External::Units::megahertz;
        using hertz     = External::Units::hertz;
        
        using namespace Project;
        
        template<uint8_t N, typename MCU>
        struct TimerBase8Bit;
        
        template<uint8_t N, AVR::Concepts::AtMega MCU>
        struct TimerBase8Bit<N, MCU> {
            using value_type = uint8_t;
            static inline constexpr uint8_t number = N;
            using mcu_timer_type = typename MCU::Timer8Bit;
            using mcu_timer_interrupts_type = typename MCU::Timer8Interrupts;
            using mcu_timer_interrupts_flags_type = typename MCU::Timer8Interrupts::Flags;
            using mcu_timer_interrupts_mask_type = typename MCU::Timer8Interrupts::Mask ;
            using ta = typename mcu_timer_type::TCCRA;
            using tb = typename mcu_timer_type::TCCRB;
            static constexpr const auto mcu_timer = getBaseAddr<mcu_timer_type, number>;
            static constexpr const auto mcu_timer_interrupts = getBaseAddr<mcu_timer_interrupts_type, number>;
        };
        
        template<AVR::Concepts::AtMega_8 MCU>
        struct TimerBase8Bit<0, MCU> {
            using value_type = uint8_t;
            static inline constexpr uint8_t number = 0;
            using mcu_timer_type = typename MCU::Timer8BitSimple;
            using mcu_timer_interrupts_type = typename MCU::TimerInterrupts;
            using mcu_timer_interrupts_flags_type = typename MCU::TimerInterrupts::Flags;
            using ta = typename mcu_timer_type::TCCR;
            using tb = void;
            static constexpr const auto mcu_timer = getBaseAddr<mcu_timer_type, number>;
            static constexpr const auto mcu_timer_interrupts = getBaseAddr<mcu_timer_interrupts_type>;
        };
        template<AVR::Concepts::AtMega_8 MCU>
        struct TimerBase8Bit<2, MCU> {
            using value_type = uint8_t;
            static inline constexpr uint8_t number = 2;
            using mcu_timer_type = typename MCU::Timer8BitSimple2;
            using mcu_timer_interrupts_type = typename MCU::TimerInterrupts;
            using mcu_timer_interrupts_flags_type = typename MCU::TimerInterrupts::Flags;
            using ta = typename mcu_timer_type::TCCR;
            using tb = void;
            static constexpr const auto mcu_timer = getBaseAddr<mcu_timer_type, number>;
            static constexpr const auto mcu_timer_nterrupts = getBaseAddr<mcu_timer_interrupts_type>;
        };
        
        template<uint8_t N, typename MCU>
        struct TimerBase16Bit {
            using value_type = uint16_t;
            static inline constexpr uint8_t number = N;
            using mcu_timer_type = typename MCU::Timer16Bit;
            using mcu_timer_interrupts_type = typename MCU::Timer16Interrupts;
            using mcu_timer_interrupts_flags_type = typename MCU::Timer16Interrupts::Flags;
            using mcu_timer_interrupts_mask_type = typename MCU::Timer16Interrupts::Mask ;
            using ta = typename mcu_timer_type::TCCRA;
            using tb = typename mcu_timer_type::TCCRB;
            static constexpr auto mcu_timer = AVR::getBaseAddr<mcu_timer_type, number>;
            static constexpr auto mcu_timer_interrupts = AVR::getBaseAddr<mcu_timer_interrupts_type, number>;
        };
        template<uint8_t N, AVR::Concepts::AtMega_8 MCU>
        struct TimerBase16Bit<N, MCU> {
            using value_type = uint16_t;
            static inline constexpr uint8_t number = N;
            using mcu_timer_type = typename MCU::Timer16Bit;
            using mcu_timer_interrupts_type = typename MCU::TimerInterrupts;
            using mcu_timer_interrupts_flags_type = typename MCU::TimerInterrupts::Flags;
            using mcu_timer_interrupts_mask_type = typename MCU::TimerInterrupts::Mask ;
            using ta = typename mcu_timer_type::TCCRA;
            using tb = typename mcu_timer_type::TCCRB;
            static constexpr auto mcu_timer = AVR::getBaseAddr<mcu_timer_type, number>;
            static constexpr auto mcu_timer_interrupts = AVR::getBaseAddr<mcu_timer_interrupts_type>;
        };
        
        template<uint8_t N, typename MCU = DefaultMcuType> struct TimerParameter;
        
        template<AVR::Concepts::AtMega_8 MCU>
        struct TimerParameter<0, MCU> final : public TimerBase8Bit<0, MCU> {
        };
        
        template<AVR::Concepts::AtMega_8 MCU>
        struct TimerParameter<2, MCU> final : public TimerBase8Bit<2, MCU> {
        };
        
        template<AVR::Concepts::AtMega_8 MCU>
        struct TimerParameter<1, MCU> final : public TimerBase16Bit<1, MCU> {
            using PortB = AVR::Port<AVR::B, MCU>;
            typedef AVR::Pin<PortB, 1> ocAPin;
            typedef AVR::Pin<PortB, 2> ocBPin;
        };
        
        
        template<AVR::Concepts::AtMega_X4 MCU>
        struct TimerParameter<0, MCU> final  : public TimerBase8Bit<0, MCU> {
            using PortB = AVR::Port<AVR::B, MCU>;
            typedef AVR::Pin<PortB, 3> ocAPin;
            typedef AVR::Pin<PortB, 4> ocBPin;
        };
        template<AVR::Concepts::AtMega_X4 MCU>
        struct TimerParameter<1, MCU> final : public TimerBase16Bit<1, MCU> {
            using PortD = AVR::Port<AVR::D, MCU>;
            typedef AVR::Pin<PortD, 5> ocAPin;
            typedef AVR::Pin<PortD, 4> ocBPin;
        };
        template<AVR::Concepts::AtMega_X4 MCU>
        struct TimerParameter<2, MCU> final  : public TimerBase8Bit<2, MCU> {
            using PortD = AVR::Port<AVR::D, MCU>;
            typedef AVR::Pin<PortD, 7> ocAPin;
            typedef AVR::Pin<PortD, 6> ocBPin;
        };
        template<AVR::Concepts::AtMega_X4 MCU>
        struct TimerParameter<3, MCU> final : public TimerBase16Bit<3, MCU> {
            using PortB = AVR::Port<AVR::B, MCU>;
            typedef AVR::Pin<PortB, 6> ocAPin;
            typedef AVR::Pin<PortB, 7> ocBPin;
        };
        template<AVR::Concepts::AtMega_X4 MCU>
        struct TimerParameter<4, MCU> final : public TimerBase16Bit<4, MCU> {
            using PortC = AVR::Port<AVR::C, MCU>;
            using PortB = AVR::Port<AVR::B, MCU>;
            typedef AVR::Pin<PortC, 4> ocAPin;
            typedef AVR::Pin<PortB, 7> ocBPin;
        };
        
        template<AVR::Concepts::AtMega_X8 MCU>
        struct TimerParameter<0, MCU> final  : public TimerBase8Bit<0, MCU> {
        };
        template<AVR::Concepts::AtMega_X8 MCU>
        struct TimerParameter<1, MCU> final : public TimerBase16Bit<1, MCU> {
            using PortB = AVR::Port<AVR::B, MCU>;
            typedef AVR::Pin<PortB, 1> ocAPin;
            typedef AVR::Pin<PortB, 2> ocBPin;
        };
        template<AVR::Concepts::AtMega_X8 MCU>
        struct TimerParameter<2, MCU> final : public TimerBase8Bit<2, MCU> {
            using PortB = AVR::Port<AVR::B, MCU>;
            using PortD = AVR::Port<AVR::D, MCU>;
            typedef AVR::Pin<PortB, 3> ocAPin;
            typedef AVR::Pin<PortD, 3> ocBPin;
        };
        template<AVR::Concepts::AtMega_X8 MCU>
        struct TimerParameter<3, MCU> final : public TimerBase16Bit<3, MCU> {
            using PortD = AVR::Port<AVR::D, MCU>;
            typedef AVR::Pin<PortD, 0> ocAPin;
            typedef AVR::Pin<PortD, 2> ocBPin;
        };
        
        template<uint8_t N>
        using mcu_timer_t = typename TimerParameter<N>::mcu_timer_type;
        
        template<uint8_t N>
        using mcu_timer_interrupts_t = typename TimerParameter<N>::mcu_timer_interrupts_type;
        
        template<uint8_t N>
        using mcu_timer_interrupts_flags_t = typename TimerParameter<N>::mcu_timer_interrupts_flags_type;
        
        template<uint8_t N>
        using mcu_timer_interrupts_mask_t = typename TimerParameter<N>::mcu_timer_interrupts_mask_type;
        
        template<uint8_t N>
        using mcu_timer_value_t = typename TimerParameter<N>::value_type;
        
        template<uint8_t N>
        using prescaler_bits_t = mcu_timer_t<N>::template PrescalerBits<N>;
        
        template<uint8_t N>
        constexpr auto prescaler_bits_v = prescaler_bits_t<N>::values;
        
    }
    
}
