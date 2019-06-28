#pragma once

#include <type_traits>

namespace AVR {
    namespace Groups {
        template<typename MCU> struct isAtMega_8 : std::false_type {};
        template<typename MCU> struct isAtMega_X8 : std::false_type {};
        template<typename MCU> struct isAtMega_X4 : std::false_type {};
        
        template<typename MCU> struct isAtMega0 : std::false_type {};

        template<typename MCU> struct isAtTiny_X4 : std::false_type {};
        template<typename MCU> struct isAtTiny_X5 : std::false_type {};
        
        template<typename MCU> struct isAtTiny1 : std::false_type {};
        
        template<> struct isAtMega_8<ATMega8> : std::true_type {};

        template<> struct isAtMega_X4<ATMega324PB> : std::true_type {};
        template<> struct isAtMega_X4<ATMega1284P> : std::true_type {};

        template<> struct isAtMega_X8<ATMega88P> : std::true_type {};
        template<> struct isAtMega_X8<ATMega168P> : std::true_type {};
        template<> struct isAtMega_X8<ATMega328P> : std::true_type {};
        template<> struct isAtMega_X8<ATMega328PB> : std::true_type {};

        template<> struct isAtMega0<ATMega4809> : std::true_type {};
        
        template<> struct isAtTiny1<ATTiny412> : std::true_type {};
        
        template<typename... PP> struct isAtMega : std::disjunction<isAtMega_8<PP>..., isAtMega_X4<PP>..., isAtMega_X8<PP>...> {};

        template<typename... PP> struct isAtTiny : std::disjunction<isAtTiny_X4<PP>..., isAtTiny_X5<PP>...> {};
    }
    
    namespace Concepts {
        namespace detail {
        }
        
        template<typename MCU>
        concept bool AtMega0 = AVR::Groups::isAtMega0<MCU>::value;

        template<typename MCU>
        concept bool AtTiny1 = AVR::Groups::isAtTiny1<MCU>::value;

        template<typename MCU>
        concept bool At01Series = (AVR::Groups::isAtTiny1<MCU>::value || AVR::Groups::isAtMega0<MCU>::value);

        template<typename MCU>
        concept bool AtMega  = AVR::Groups::isAtMega<MCU>::value;
        
        template<typename MCU>
        concept bool AtMega_8 = AVR::Groups::isAtMega_8<MCU>::value;

        template<typename MCU>
        concept bool AtMega_X8 = AVR::Groups::isAtMega_X8<MCU>::value;

        template<typename MCU>
        concept bool AtMega_X4 = AVR::Groups::isAtMega_X4<MCU>::value;

        template<typename MCU>
        concept bool AtTiny_X4 = AVR::Groups::isAtTiny_X4<MCU>::value;

        template<typename MCU>
        concept bool AtTiny_X5 = AVR::Groups::isAtTiny_X5<MCU>::value;
        
        template<typename L>
        concept bool Letter = std::is_same_v<typename L::value_type, char>;
        
        template<typename C>
        concept bool McuSingleComponent = requires(C) {
                C::address;
        };
        
        template<typename C>
        concept bool McuMultipleComponent = requires(C) {
                C::count;
//                C::template Address<0>::value; // not possible beacuse of timer numbering scheme 
        } || requires(C) {
                AVR::Component::Count<C>::value;
        };
        
        template<typename P>
        concept bool McuPart = std::is_same_v<typename P::value_type, char> && ((P::value >= 'A') && (P::value <= 'H'));

        template<typename C>
        concept bool ComponentNumber = requires(C) {
            typename C::value_type;
            C::value;
        };        

        template<typename C>
        concept bool ComponentSpecifier = requires(C) {
            typename C::component_type;
//            typename C::value_type;
//            C::value_type::value;
        };        

        template<typename CP>
        concept bool ComponentPosition = requires(CP) {
            typename CP::component_type;
            typename CP::place_type;
//            typename C::value_type;
//            C::value_type::value;
        };        
        
        template<typename P>
        concept bool Port = requires (P p) { 
                typename P::mcuport_type;
                typename P::name_type;
                p.get();
        };
        
        template<typename P>
        concept bool Pin = std::is_same_v<P, void> || requires (P p) { 
                p.on();
                p.off();
        };
        
        template<typename I>
        concept bool IServiceRNonVoid = requires (I) {
                I::isr();
                I::isr_number;
        };
        template<typename I>
        concept bool IServiceR = std::is_same_v<I, void> || IServiceRNonVoid<I>;
        
        template<typename I>
        concept bool Interrupt = std::is_same_v<I, void>|| requires (I i) {
                I::number;
        };
        
        template<typename D>
        concept bool Device = requires(D) {
                D::put(std::byte{0});
        };

        template<typename S>
        concept bool Stream = requires(S s) { 
                typename S::device_type;
                typename S::line_terminator_type;
        };

        template<typename PA>
        concept bool ProtocolAdapter = requires(PA) {
                PA::process(std::byte{0});
        };

        template<typename AC>
        concept bool Actor = requires(AC) {
                typename AC::value_type;     
                AC::set(typename AC::value_type{});
        };
        
    }
}
