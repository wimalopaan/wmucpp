#pragma once

#include <std/chrono>

#include <etl/rational.h>

#include "groups.h"
#include "../common/ppm.h"

namespace AVR {
    using namespace std::literals::chrono;
    using namespace External::Units;
    using namespace AVR::Util::Timer;

    template<auto TimerNumber, typename Out, auto Channels, typename Multiplexer = void, typename DB = void, typename MCU = DefaultMcuType>
    struct Cppm final {
        Cppm() = delete;
        using mcu_timer_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_type;
        using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;        
        using ta = typename TimerParameter<TimerNumber, MCU>::ta;        
        using tb = typename TimerParameter<TimerNumber, MCU>::tb;        

        using flags_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_interrupts_flags_type;
        
        using ocPin = std::conditional_t<std::is_same_v<Out, AVR::A>, 
                                        typename TimerParameter<TimerNumber, MCU>::ocAPin, 
                                        typename TimerParameter<TimerNumber, MCU>::ocBPin>;
    
        using parameter = PPMParameter<TimerNumber>;
        using ranged_type = etl::uint_ranged<uint16_t, parameter::ocMin, parameter::ocMax> ;
        using index_type  = etl::uint_ranged<uint8_t, 0, Channels - 1>;        

        static constexpr auto mcu_timer = TimerParameter<TimerNumber, MCU>::mcu_timer;
        static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber, MCU>::mcu_timer_interrupts;
        
        inline static constexpr void init() {
            Multiplexer::init();
            ocPin::template dir<AVR::Output>();
            
            constexpr auto bits = bitsFrom<parameter::prescaler>(mcu_timer_type::template PrescalerBits<TimerNumber>::values);
            static_assert(isset(bits), "wrong prescaler");
            mcu_timer()->tccrb.template set<bits>();
            
            // mode 14
            mcu_timer()->tccra.template set<ta::wgm1 | ta::coma1 | ta::comb1>();
            mcu_timer()->tccrb.template add<tb::wgm2 | tb::wgm3, etl::DisbaleInterrupt<etl::NoDisableEnable>>();

            *mcu_timer()->icr = parameter::ccPulseFrame;
            *mcu_timer()->ocra = parameter::ocMedium;
            *mcu_timer()->ocrb = parameter::ocMedium;
        }
        
        inline static constexpr void periodic() {
            // wenn ocfa (ocfb) gesetzt, dann neuen Wert in ocra (wird bei Bottom updedatet)
            if (mcu_timer_interrupts()->tifr.template isSet<flags_type::ocfa>()) {
                mcu_timer_interrupts()->tifr.template reset<flags_type::ocfa>();
                ++index;
                Multiplexer::select(index.toRanged());
                *mcu_timer()->ocra = values[index.toInt()];
            }
        }

        template<typename T, auto L, auto U>
        static void ppm(index_type ch, etl::uint_ranged_NaN<T, L, U> raw) {
            if (raw) {
                T v1 = raw.toInt() - L;
                constexpr uint64_t denom = U - L;
                constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
                if constexpr(nom < denom) {
                    uint16_t ocr = etl::Rational::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
                    values[ch] = clamp(ocr, parameter::ocMin, parameter::ocMax);
                }
                else {
                    static_assert( (((10 * nom) / denom) * 255) <= std::numeric_limits<uint16_t>::max());
                    uint16_t ocr = ((v1 * ((10 * nom) / denom)) / 10) + ranged_type::Lower;
                    values[ch] = clamp(ocr, parameter::ocMin, parameter::ocMax);
                }
            }
        }
    private:
        inline static auto values = [](){
                std::array<value_type, Channels> a;
                std::fill(std::begin(a), std::end(a), parameter::ocMedium);
                return a;
         }();
        
        inline static etl::uint_ranged_circular<uint8_t, 0, values.size() - 1> index;        
    };
}
