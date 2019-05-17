#pragma once

#include "concepts.h"

namespace AVR {
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
        struct TimerBase8Bit {
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
