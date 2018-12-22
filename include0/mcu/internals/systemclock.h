#pragma once

#include <std/chrono>

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    
    template<int Number, const std::chrono::milliseconds& Interval, typename MCU = DefaultMcuType>
    struct SystemTimer {
        static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8Bit, Number>;
        static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::Timer8Interrupts, Number>;
        
        typedef typename MCU::Timer8Bit mcu_timer_type;
        
        typedef typename MCU::Timer8Bit::TCCRB tccrb_type;
        typedef typename MCU::Timer8Interrupts::Flags flags_type;
        typedef uint8_t value_type;
        
        static constexpr auto intervall = Interval;
        static constexpr auto frequency = uint16_t{1} / Interval;
        static constexpr auto tsd = AVR::Util::Timer::calculate<mcu_timer_type, Number>(frequency);
        static_assert(tsd, "falscher wert für p");
        
        template<int PreScale>
        inline static void prescale() {
            constexpr tccrb_type bits = AVR::Util::Timer::bitsFrom<PreScale>(MCU::Timer8Bit::template PrescalerBits<Number>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcuTimer()->tccrb.template set<bits>();
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
        }
        
        inline static void init() {
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
            prescale<tsd.prescaler>();
            *mcuTimer()->ocra = tsd.ocr - 1;
        }

        template<etl::Concepts::Callable Callable>
        inline static void periodic(const Callable& f) {
            if (mcuInterrupts()->tifr.template isSet<flags_type::ocfa>()) {
                f();
                mcuInterrupts()->tifr.template reset<flags_type::ocfa>(); // reset
            } 
        }
        
    private:
    };
}
