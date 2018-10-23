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

#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/groups.h"
#include "mcu/avr/ppmbase.h"
#include "mcu/avr/util.h"
#include "units/percent.h"
#include "util/disable.h"
#include "util/rational.h"

namespace AVR {
    template<uint8_t TimerN, typename Out = A, uint8_t NChannels = 8, 
             typename DebugPin = void, typename Prescaler = void, typename MCU = DefaultMcuType>
    class CPPM final {
        CPPM() = delete;        
    public:
        typedef MCU mcu_type;
        typedef typename AVR::TimerParameter<TimerN, MCU>::timer_type timer_type;
        using MCUTimer = typename timer_type::mcu_timer_type;
        
        using ocPin = std::conditional_t<std::is_same_v<Out, AVR::A>, typename AVR::TimerParameter<TimerN, MCU>::ocAPin, typename AVR::TimerParameter<TimerN, MCU>::ocBPin>;
        
        static constexpr const auto mcuTimer = AVR::getBaseAddr<MCUTimer, TimerN>;
        
        using parameter = PPMParameter<typename AVR::TimerParameter<TimerN, MCU>::timer_type, Prescaler>;
        
        typedef uint_ranged<uint16_t, parameter::ocMin, parameter::ocMax> ranged_type;

        typedef uint_ranged<uint8_t, 0, NChannels - 1> index_type;        

        inline static constexpr typename timer_type::value_type medium = (parameter::ocMin + parameter::ocMax) / 2;
        
        static void init() {
            if constexpr(!std::is_same_v<DebugPin, void>) {
                DebugPin::template dir<Output>();
            }

            ocPin::template dir<AVR::Output>();
            if constexpr(TimerN >= 3) { // Output Compare Modulator OCM
                using ocBPin = typename AVR::TimerParameter<TimerN, MCU>::ocBPin;
                ocBPin::on();                
            }
            
            timer_type::template prescale<parameter::prescaler>();
            
            *mcuTimer()->icr = parameter::ocFrame;
            
            if constexpr(std::is_same<Prescaler, void>::value) {
                mcuTimer()->tccra.template set<AVR::TimerParameter<TimerN, MCU>::FastPwm2::tccra>();
                mcuTimer()->tccrb.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm2::tccrb, DisbaleInterrupt<NoDisableEnable>>();
            }
            else {
                mcuTimer()->tccra.template set<AVR::TimerParameter<TimerN, MCU>::FastPwm1::tccra>();
                mcuTimer()->tccrb.template add<AVR::TimerParameter<TimerN, MCU>::FastPwm1::tccrb, DisbaleInterrupt<NoDisableEnable>>();
            }
//            *mcuTimer()->ocra = medium;
            *mcuTimer()->ocrb = parameter::ccPulse;
        }
        static void ppm(index_type ch, const std::percent& width) {
            uint16_t ocr = std::expand(width, (uint32_t)parameter::ocMin, (uint32_t)parameter::ocMax);
            values[ch] = ocr;
        }
        template<typename T, auto IL, auto IU, auto L, auto U>
        static void ppm(uint_ranged<uint8_t, IL, IU> ch, uint_ranged<T, L, U> raw) {
            static_assert(IL >= index_type::Lower);
            static_assert(IU <= index_type::Upper);
            ppm(index_type{ch.toInt()}, raw);
        }
        template<typename T, auto L, auto U>
        static void ppm(index_type ch, uint_ranged<T, L, U> raw) {
            T v1 = raw.toInt() - L;
            constexpr uint64_t denom = U - L;
            constexpr uint64_t nom = ranged_type::Upper - ranged_type::Lower;
            if constexpr(nom < denom) {
                uint16_t ocr = ::Util::RationalDivider<uint16_t, nom, denom>::scale(v1) + ranged_type::Lower;
                values[ch] = ocr;
            }
            else {
                static_assert( (((10 * nom) / denom) * 255) <= std::numeric_limits<uint16_t>::max());
                uint16_t ocr = ((v1 * ((10 * nom) / denom)) / 10) + ranged_type::Lower;
                values[ch] = ocr;
            }
        }
        template<typename T, auto L, auto U>
        static void ppm(index_type ch, uint_ranged_NaN<T, L, U> raw) {
            if (raw) {
                ppm(index, uint_ranged<T, L, U>{raw});
            }
        }
        inline static void periodic() {
            timer_type::template periodic<timer_type::flags_type::ocfb>([](){
                if constexpr(!std::is_same_v<DebugPin, void>) {
                    if (index == 0) {
                        DebugPin::toggle();
                    }
                }
                *mcuTimer()->icr = values[index];
                ++index;
            });
        }
    private:
        inline static auto values = [](){
                std::array<typename AVR::TimerParameter<TimerN, MCU>::value_type, NChannels + 1> a;
                std::fill(std::begin(a), std::end(a), medium);
                a[NChannels] = parameter::ocMax + 4 * parameter::ccPulse;
                return a;
         }();
        inline static uint_ranged_circular<uint8_t, 0, values.size - 1> index;        
    };
}
