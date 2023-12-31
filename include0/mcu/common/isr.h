/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <array>

#include "concepts.h"

namespace AVR {
    
    template<uint8_t N>
    using ExternalInterruptNumber = etl::NamedConstant<N>;
    
    template<AVR::Concepts::IServiceR... HH>
    struct IsrRegistrar {
        typedef IsrRegistrar type;
        
        typedef Meta::List<HH...> handlers;
        typedef Meta::filter<Meta::nonVoid, handlers> nonVoidHandlers;
        
        template<typename I> using getIsr = typename I::isr_type;
        typedef Meta::transform<getIsr, nonVoidHandlers> interrupts;
        
        template<AVR::Concepts::Interrupt I>
        static constexpr bool isSet = Meta::contains<interrupts, I>::value;
        
        template<AVR::Concepts::Interrupt INT>
        struct IsrCaller {
            static_assert(isSet<INT>, "isr not set");

            template<AVR::Concepts::IServiceR Int, auto number>
            struct hasNumber : std::integral_constant<bool, Int::isr_number == number> {};

            template<AVR::Concepts::IServiceR I>
            using hasNumberInt = hasNumber<I, INT::number>;

            using isr_type_list = Meta::filter<hasNumberInt, nonVoidHandlers>;
            static_assert(Meta::size<isr_type_list>::value == 1, "double defined isr for the same interrupt");

            using isr_type = Meta::front<isr_type_list>;
            
            inline static void isr() {
                isr_type::isr();    
            }
        };
        
        template<AVR::Concepts::Interrupt INT>
        inline static void isr() {
            IsrCaller<INT>::isr();            
        }
    };
    
    template<AVR::Concepts::Interrupt I>
    struct IsrBaseHandler {
        typedef I isr_type;
        static constexpr const uint8_t isr_number = I::number;
    };
    struct NoIsrBaseHandler {};
    
    template<AVR::Concepts::Interrupt I, AVR::Concepts::IServiceR... Hs>
    struct IsrDistributor final : public IsrBaseHandler<I> {
        typedef Meta::List<Hs...> handlers;
        typedef Meta::filter<Meta::nonVoid, handlers> nonVoidHandlers;
        
        template<typename T> using getIsr = typename T::isr_type;
        typedef Meta::transform<getIsr, nonVoidHandlers> interrupts;
        
        template<typename T> using usesSameInterrupt = std::is_same<I, T>;
        typedef Meta::filter<usesSameInterrupt, interrupts> listOfSameInterrupts;
        
        static_assert(Meta::size<listOfSameInterrupts>::value == Meta::size<interrupts>::value , "all sub-handler must use same isr");
        
        inline static void isr() {
            Caller<nonVoidHandlers>::isr();
        }
    private:
        template<typename L> struct Caller;
        template<typename... NVHH>
        struct Caller<Meta::List<NVHH...>> {
            inline static void isr() {
                (NVHH::isr(), ...);
            }  
        };
    };
    
    namespace ISR {
        
        struct NoInterrupt {};
        
        template<uint8_t N>
        struct Int;

        // integral_constant        
#ifdef INT1_vect_num
        template<>
        struct Int<0> {
            static constexpr const uint32_t number = INT0_vect_num;
        };
#endif
#ifdef INT1_vect_num
        template<>
        struct Int<1> {
            static constexpr const uint32_t number = INT1_vect_num;
        };
#endif
#ifdef INT2_vect_num
        template<>
        struct Int<2> {
            static constexpr const uint32_t number = INT2_vect_num;
        };
#endif

        template<AVR::Concepts::Letter L>
        struct Port;
#ifdef PORTA_PORT_vect_num
        template<>
        struct Port<A> {
            inline static constexpr uint32_t number = PORTA_PORT_vect_num;
        };
#endif
#ifdef PORTB_PORT_vect_num
        template<>
        struct Port<B> {
            inline static constexpr uint32_t number = PORTB_PORT_vect_num;
        };
#endif
#ifdef PORTC_PORT_vect_num
        template<>
        struct Port<C> {
            inline static constexpr uint32_t number = PORTC_PORT_vect_num;
        };
#endif
#ifdef PORTD_PORT_vect_num
        template<>
        struct Port<D> {
            inline static constexpr uint32_t number = PORTD_PORT_vect_num;
        };
#endif
#ifdef PORTE_PORT_vect_num
        template<>
        struct Port<E> {
            inline static constexpr uint32_t number = PORTE_PORT_vect_num;
        };
#endif
#ifdef PORTF_PORT_vect_num
        template<>
        struct Port<F> {
            inline static constexpr uint32_t number = PORTF_PORT_vect_num;
        };
#endif
        
        template<uint8_t N>
        struct PcInt;
#ifdef PCINT0_vect_num
        template<>
        struct PcInt<0>{
            static constexpr const uint32_t number = PCINT0_vect_num;
        };
#endif
#ifdef PCINT1_vect_num
        template<>
        struct PcInt<1>  {
            static constexpr const uint32_t number = PCINT1_vect_num;
        };
#endif
#ifdef PCINT2_vect_num
        template<>
        struct PcInt<2>  {
            static constexpr const uint32_t number = PCINT2_vect_num;
        };
#endif
#ifdef PCINT3_vect_num
        template<>
        struct PcInt<3>  {
            static constexpr const uint32_t number = PCINT3_vect_num;
        };
#endif
#ifdef WDT_vect_num
        struct Wdt  {
            static constexpr const uint32_t number = WDT_vect_num;
        };
#endif

        template<uint8_t N>
        struct Tca;
        
        template<>
        struct Tca<0> {
#ifdef TCA0_LUNF_vect_num
            struct Lunf
            {
                static constexpr const uint32_t number = TCA0_LUNF_vect_num;
            };
#endif
#ifdef TCA0_OVF_vect_num
            struct Ovf
            {
                static constexpr const uint32_t number = TCA0_OVF_vect_num;
            };
#endif
#ifdef TCA0_CMP0_vect_num
            template<uint8_t CC>
            struct Cmp
            {
                static constexpr const uint32_t number = TCA0_CMP0_vect_num + CC;
            };
#endif
        };
        
        template<uint8_t N>
        struct Tcb;
        
        template<>
        struct Tcb<0> {
#ifdef TCB0_INT_vect_num
            struct Capture
            {
                static constexpr const uint32_t number = TCB0_INT_vect_num;
            };
#endif
        };
        template<>
        struct Tcb<1> {
#ifdef TCB1_INT_vect_num
            struct Capture
            {
                static constexpr const uint32_t number = TCB1_INT_vect_num;
            };
#endif
        };
        template<>
        struct Tcb<2> {
#ifdef TCB2_INT_vect_num
            struct Capture
            {
                static constexpr const uint32_t number = TCB2_INT_vect_num;
            };
#endif
        };

        template<uint8_t N>
        struct Tcd;
        
        template<>
        struct Tcd<0> {
#ifdef TCD0_OVF_vect_num
            struct Ovf
            {
                static constexpr const uint32_t number = TCD0_OVF_vect_num;
            };
#endif
        };
        
        template<uint8_t N>
        struct Timer;
        
        template<>
        struct Timer<4> {
            struct Capture  {
#ifdef TIMER4_CAPT_vect_num
                static constexpr const uint32_t number = TIMER4_CAPT_vect_num;
#endif
            };
            struct CompareA  {
#ifdef TIMER4_COMPA_vect_num
                static constexpr const uint32_t number = TIMER4_COMPA_vect_num;
#endif
            };
            struct CompareB  {
#ifdef TIMER4_COMPB_vect_num
                static constexpr const uint32_t number = TIMER4_COMPB_vect_num;
#endif
            };
            struct Compare  {
#ifdef TIMER4_COMP_vect_num
                static constexpr const uint32_t number = TIMER4_COMP_vect_num;
#endif
            };
            struct Overflow  {
#ifdef TIMER4_OVF_vect_num
                static constexpr const uint32_t number = TIMER4_OVF_vect_num;
#endif
            };
        };
        template<>
        struct Timer<3> {
            struct Capture  {
#ifdef TIMER3_CAPT_vect_num
                static constexpr const uint32_t number = TIMER3_CAPT_vect_num;
#endif
            };
            struct CompareA  {
#ifdef TIMER3_COMPA_vect_num
                static constexpr const uint32_t number = TIMER3_COMPA_vect_num;
#endif
            };
            struct CompareB  {
#ifdef TIMER3_COMPB_vect_num
                static constexpr const uint32_t number = TIMER3_COMPB_vect_num;
#endif
            };
            struct Compare  {
#ifdef TIMER3_COMP_vect_num
                static constexpr const uint32_t number = TIMER3_COMP_vect_num;
#endif
            };
            struct Overflow  {
#ifdef TIMER3_OVF_vect_num
                static constexpr const uint32_t number = TIMER3_OVF_vect_num;
#endif
            };
        };
        template<>
        struct Timer<2> {
            struct CompareA  {
#ifdef TIMER2_COMPA_vect_num
                static constexpr const uint32_t number = TIMER2_COMPA_vect_num;
#endif
            };
            struct CompareB  {
#ifdef TIMER2_COMPB_vect_num
                static constexpr const uint32_t number = TIMER2_COMPB_vect_num;
#endif
            };
            struct Compare  {
#ifdef TIMER2_COMP_vect_num
                static constexpr const uint32_t number = TIMER2_COMP_vect_num;
#endif
            };
            struct Overflow  {
#ifdef TIMER2_OVF_vect_num
                static constexpr const uint32_t number = TIMER2_OVF_vect_num;
#endif
            };
        };
        template<>
        struct Timer<1> {
            struct Capture  {
#ifdef TIMER1_CAPT_vect_num
                static constexpr const uint32_t number = TIMER1_CAPT_vect_num;
#elif defined(TIM1_CAPT_vect_num)
                static constexpr const uint32_t number = TIM1_CAPT_vect_num;
#endif
            };
            struct CompareA  {
#ifdef TIMER1_COMPA_vect_num
                static constexpr const uint32_t number = TIMER1_COMPA_vect_num;
#elif defined(TIM1_COMPA_vect_num)
                static constexpr const uint32_t number = TIM1_COMPA_vect_num;
#endif
            };
            struct CompareB  {
#ifdef TIMER1_COMPB_vect_num
                static constexpr const uint32_t number = TIMER1_COMPB_vect_num;
#elif defined(TIM1_COMPB_vect_num)
                static constexpr const uint32_t number = TIM1_COMPB_vect_num;
#endif
            };
            struct Overflow  {
#ifdef TIMER1_OVF_vect_num
                static constexpr const uint32_t number = TIMER1_OVF_vect_num;
#endif
            };
        };
        template<>
        struct Timer<0> {
            struct CompareA  {
#ifdef TIMER0_COMPA_vect_num
                static constexpr const uint32_t number = TIMER0_COMPA_vect_num;
#endif
            };
            struct CompareB  {
#ifdef TIMER0_COMPB_vect_num
                static constexpr const uint32_t number = TIMER0_COMPB_vect_num;
#endif
            };
            struct Overflow  {
#ifdef TIMER0_OVF_vect_num
                static constexpr const uint32_t number = TIMER0_OVF_vect_num;
#endif
            };
        };
        
        template<uint8_t N>
        struct Spi;
        
        template<>
        struct Spi<0> {
            struct Stc  {
#ifdef SPI_STC_vect_num
                static constexpr const uint32_t number = SPI_STC_vect_num;
#endif
#ifdef SPI0_STC_vect_num
                static constexpr const uint32_t number = SPI0_STC_vect_num;
#endif
            };
        };
        template<>
        struct Spi<1> {
            struct Stc  {
#ifdef SPI1_STC_vect_num
                static constexpr const uint32_t number = SPI1_STC_vect_num;
#endif
            };
        };
        
        template<uint8_t> 
        struct Usi;
        
        template<>
        struct Usi<0> {
            struct Overflow {
#ifdef USI_OVF_vect_num
                static constexpr const uint32_t number = USI_OVF_vect_num;
#endif
            };
            struct Start {
#ifdef USI_STR_vect_num
                static constexpr const uint32_t number = USI_STR_vect_num;
#elif defined(USI_START_vect_num)
                static constexpr const uint32_t number = USI_START_vect_num;
#endif
            };
        };
        
        template<uint8_t>
        struct Twi;
        template<>
        struct Twi<0> {
#ifdef TWI_vect_num
            static constexpr const uint32_t number = TWI_vect_num;
#elif defined(TWI0_vect_num)
            static constexpr const uint32_t number = TWI0_vect_num;
#endif
        };
        template<>
        struct Twi<1> {
#ifdef TWI1_vect_num
            static constexpr const uint32_t number = TWI1_vect_num;
#endif
        };
        
        template<uint8_t> struct Usart;
        
        template<> struct Usart<0> {
#ifdef USART_RX_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART_RX_vect_num;
            };
#endif
#ifdef USART_RXC_vect_num
#undef RXC
            struct RXC {
                static constexpr const uint32_t number = USART_RXC_vect_num;
            };
#endif
#ifdef USART0_RX_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART0_RX_vect_num;
            };
#endif
            struct UDREmpty {
#ifdef USART_UDRE_vect_num
                static constexpr const uint32_t number = USART_UDRE_vect_num;
#endif
#ifdef USART0_UDRE_vect_num
                static constexpr const uint32_t number = USART0_UDRE_vect_num;
#endif
            };
#ifdef USART0_DRE_vect_num
            struct DRE {
                static constexpr const uint32_t number = USART0_DRE_vect_num;
            };
#endif
#ifdef USART0_TXC_vect_num
            struct TXC {
                static constexpr const uint32_t number = USART0_TXC_vect_num;
            };
#endif
#ifdef USART0_RXC_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART0_RXC_vect_num;
            };
#endif
        };
        template<>
        struct Usart<1> {
#ifdef USART1_RX_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART1_RX_vect_num;
            };
#endif
#ifdef USART1_UDRE_vect_num
            struct UDREmpty {
                static constexpr const uint32_t number = USART1_UDRE_vect_num;
            };
#endif
#ifdef USART1_DRE_vect_num
            struct DRE {
                static constexpr const uint32_t number = USART1_DRE_vect_num;
            };
#endif
#ifdef USART1_TXC_vect_num
            struct TXC {
                static constexpr const uint32_t number = USART1_TXC_vect_num;
            };
#endif
#ifdef USART1_RXC_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART1_RXC_vect_num;
            };
#endif
        };
        template<>
        struct Usart<2> {
            struct RX {
#ifdef USART2_RX_vect_num
                static constexpr const uint32_t number = USART2_RX_vect_num;
#endif
            };
            struct UDREmpty {
#ifdef USART2_UDRE_vect_num
                static constexpr const uint32_t number = USART2_UDRE_vect_num;
#endif
            };
#ifdef USART2_DRE_vect_num
            struct DRE {
                static constexpr const uint32_t number = USART2_DRE_vect_num;
            };
#endif
#ifdef USART2_TXC_vect_num
            struct TXC {
                static constexpr const uint32_t number = USART2_TXC_vect_num;
            };
#endif
#ifdef USART2_RXC_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART2_RXC_vect_num;
            };
#endif
        };
        template<>
        struct Usart<3> {
            struct RX {
#ifdef USART3_RX_vect_num
                static constexpr const uint32_t number = USART3_RX_vect_num;
#endif
            };
            struct UDREmpty {
#ifdef USART3_UDRE_vect_num
                static constexpr const uint32_t number = USART3_UDRE_vect_num;
#endif
            };
#ifdef USART3_DRE_vect_num
            struct DRE {
                static constexpr const uint32_t number = USART3_DRE_vect_num;
            };
#endif
#ifdef USART3_TXC_vect_num
            struct TXC {
                static constexpr const uint32_t number = USART3_TXC_vect_num;
            };
#endif
#ifdef USART3_RXC_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART3_RXC_vect_num;
            };
#endif
        };


        template<>
        struct Usart<4> {
            struct RX {
#ifdef USART4_RX_vect_num
                static constexpr const uint32_t number = USART4_RX_vect_num;
#endif
            };
            struct UDREmpty {
#ifdef USART4_UDRE_vect_num
                static constexpr const uint32_t number = USART4_UDRE_vect_num;
#endif
            };
#ifdef USART4_DRE_vect_num
            struct DRE {
                static constexpr const uint32_t number = USART4_DRE_vect_num;
            };
#endif
#ifdef USART4_TXC_vect_num
            struct TXC {
                static constexpr const uint32_t number = USART4_TXC_vect_num;
            };
#endif
#ifdef USART4_RXC_vect_num
            struct RXC {
                static constexpr const uint32_t number = USART4_RXC_vect_num;
            };
#endif
        };

        
        template<uint8_t> 
        struct AdComparator;
        
        template<> 
        struct AdComparator<0> {
#ifdef ANALOG_COMP_vect_num
            struct Edge {
                static constexpr const uint32_t number = ANALOG_COMP_vect_num;
            };
#endif
#ifdef AC0_AC_vect_num
            struct Edge {
                static constexpr const uint32_t number = AC0_AC_vect_num;
            };
#endif
        };
        
    }
}
