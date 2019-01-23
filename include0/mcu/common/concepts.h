#pragma once

#include <type_traits>

namespace AVR {
    namespace Groups {
        template<typename MCU> struct isAtMega_X8;
        template<typename MCU> struct isAtMega_X4;
    }
    namespace Concepts {
        
        template<typename MCU>
        concept bool AtMega_X8 = AVR::Groups::isAtMega_X8<MCU>::value;

        template<typename MCU>
        concept bool AtMega_X4 = AVR::Groups::isAtMega_X4<MCU>::value;
        
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
        };
        
        template<typename P>
        concept bool Port = requires (P p) { 
                typename P::mcuport_type;
                typename P::name_type;
                p.get();
        };
        
        template<typename P>
        concept bool Pin = std::is_same<P, void>::value || requires (P p) { 
                p.on();
                p.off();
        };
        
        template<typename I>
        concept bool IServiceRNonVoid = requires (I) {
                I::isr();
                I::isr_number;
        };
        template<typename I>
        concept bool IServiceR = std::is_same<I, void>::value || IServiceRNonVoid<I>;
        
        template<typename I>
        concept bool Interrupt = std::is_same<I, void>::value || requires (I i) {
                I::number;
        };
        
        template<typename D>
        concept bool Device = requires(D) {
                D::put(std::byte{0});
        };

        template<typename S>
        concept bool Stream = requires (S s) { 
                typename S::device_type;
                typename S::line_terminator_type;
        };
    
    }
}
