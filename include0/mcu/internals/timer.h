#pragma once

#include "../common/timer.h"
#include "../common/isr.h"

#include <std/chrono>

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;
    
    namespace Util::Timer {
        // calculates prescaler and ocr value for frequency with lowest possible prescaler
        template<uint8_t TimerNumber, typename MCU = DefaultMcuType>
        constexpr auto calculate(const hertz& ftimer) {
            using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;  
            using pBits = prescaler_bits_t<TimerNumber>;
            using tsd_type = TimerSetupData<value_type>;
            
            auto prescalers = prescalerValues(pBits::values);
            
            for(const auto& p : etl::sort(prescalers)) { // aufsteigend
                if (p > 0) {
                    const auto tv = (Project::Config::fMcu / ftimer) / p;
                    if ((tv > 0) && (tv < std::numeric_limits<value_type>::max())) {
                        const bool exact = ((Project::Config::fMcu.value / p) % tv) == 0;
                        return tsd_type{p, static_cast<value_type>(tv), Project::Config::fMcu / tv / uint32_t(p), exact};
                    }
                }
            }
            return tsd_type{};
        }
    }    
    
    template<auto Number, typename MCU = DefaultMcuType>
    struct ExtendedTimer {
        using mcu_timer_type = mcu_timer_t<Number>;
        using tccra_type = typename TimerParameter<Number, MCU>::ta;        
        
        static constexpr auto number = Number;
        static constexpr auto mcu_timer = TimerParameter<Number, MCU>::mcu_timer;
        
        struct OVFHandler : public AVR::IsrBaseHandler<typename AVR::ISR::Timer<Number>::Overflow> {
            inline static void isr() {
                ++ovlCounter;
            }
        }; 
        inline static void init() {
            // enable overflow
            mcu_timer()->tccr.template add<tccra_type::wgm1>();
        }
        inline static uint16_t value() {
            return (ovlCounter << 8) + *mcu_timer()->tcnt;
        }
        template<uint16_t PreScale>
        inline static void prescale() {
            constexpr auto bits = AVR::Util::Timer::bitsFrom<PreScale>(mcu_timer_type::template PrescalerBits<Number>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcu_timer()->tccr.template set<bits>();
        }
    private:
        static inline volatile uint8_t ovlCounter = 0;
    };
    
    template<typename ComponentNumber, typename MCU = DefaultMcuType>
    struct SimpleTimer;
    
    template<uint8_t N, AVR::Concepts::At01Series MCU>
    struct SimpleTimer<AVR::Component::Tcb<N>, MCU> {
        using value_type = uint16_t;
        
        static inline constexpr auto mcu_tcb = AVR::getBaseAddr<typename MCU::TCB, N>;
        
        using ctrla_t = MCU::TCB::CtrlA_t;
        using ctrlb_t = MCU::TCB::CtrlB_t;
        using intflags_t = MCU::TCB::IntFlags_t;
        
        inline static constexpr auto on_flags  = ctrla_t::clkdiv1 | ctrla_t::enable;
        inline static constexpr auto off_flags = ctrla_t::clkdiv1;
        
        inline static void init() {
            mcu_tcb()->ctrlb.template set<ctrlb_t::mode_int>();
            mcu_tcb()->ctrla.template set<on_flags>();
            mcu_tcb()->intflags.template reset<intflags_t::capt>();
        }
        
        inline static void periodic(auto f) {
            if (mcu_tcb()->intflags.template isSet<intflags_t::capt>()) {
                f();
                mcu_tcb()->intflags.template reset<intflags_t::capt>();
            }
        }
        
        inline static void off() {
            mcu_tcb()->ctrla.template set<off_flags>();
        }

        inline static void on() {
            mcu_tcb()->intflags.template reset<intflags_t::capt>();
            *mcu_tcb()->cnt = 0;
            mcu_tcb()->ctrla.template set<on_flags>();
        }
        
        inline static void period(value_type p) {
            off();
            *mcu_tcb()->ccmp = p;
            on();
        }
        
        inline static value_type counter() {
            return *mcu_tcb()->cnt;
        }
    };
}
