#pragma once

namespace AVR {
    namespace Groups {
        
        template<typename MCU>
        struct isAtMega_X4 : std::false_type {};

        template<> struct isAtMega_X4<ATMega324PB> : std::true_type {};
        template<> struct isAtMega_X4<ATMega1284P> : std::true_type {};

        template<typename MCU>
        struct isAtMega_X8 : std::false_type {};

        template<> struct isAtMega_X8<ATMega88P> : std::true_type {};
        template<> struct isAtMega_X8<ATMega168P> : std::true_type {};
        template<> struct isAtMega_X8<ATMega328P> : std::true_type {};
        template<> struct isAtMega_X8<ATMega328PB> : std::true_type {};
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
            using ta = typename mcu_timer_type::TCCRA;
            using tb = typename mcu_timer_type::TCCRB;
            static constexpr const auto mcuTimer = getBaseAddr<mcu_timer_type, number>;
        };
        
        template<uint8_t N, typename MCU>
        struct TimerBase16Bit {
            using value_type = uint16_t;
            static inline constexpr uint8_t number = N;
            using mcu_timer_type = typename MCU::Timer16Bit;
            using mcu_timer_interrupts_type = typename MCU::Timer16Interrupts;
            using mcu_timer_interrupts_flags_type = typename MCU::Timer16Interrupts::Flags;
            using ta = typename mcu_timer_type::TCCRA;
            using tb = typename mcu_timer_type::TCCRB;
            static constexpr auto mcu_timer = AVR::getBaseAddr<mcu_timer_type, number>;
            static constexpr auto mcu_timer_interrupts = AVR::getBaseAddr<mcu_timer_interrupts_type, number>;
        };
        
        template<uint8_t N, typename MCU> struct TimerParameter;

        template<AVR::Concepts::AtMega_X4 MCU>
        struct TimerParameter<0, MCU> final {
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
        template<AVR::Concepts::AtMega_X8 MCU>
        struct TimerParameter<0, MCU> final {
            static inline constexpr uint8_t number = 0;
            using mcu_timer_type = typename MCU::Timer8Bit;
            using value_type = uint8_t;
            using ta = typename mcu_timer_type::TCCRA;
            using tb = typename mcu_timer_type::TCCRB;
            static constexpr const auto mcu_timer = getBaseAddr<mcu_timer_type, number>;
        };
        template<AVR::Concepts::AtMega_X8 MCU>
        struct TimerParameter<1, MCU> final : public TimerBase16Bit<1, MCU> {
            using PortB = AVR::Port<AVR::B, MCU>;
            typedef AVR::Pin<PortB, 1> ocAPin;
            typedef AVR::Pin<PortB, 2> ocBPin;
        };
        template<AVR::Concepts::AtMega_X8 MCU>
        struct TimerParameter<3, MCU> final : public TimerBase16Bit<3, MCU> {
            using PortD = AVR::Port<AVR::D, MCU>;
            typedef AVR::Pin<PortD, 0> ocAPin;
            typedef AVR::Pin<PortD, 2> ocBPin;
        };
    }
}
