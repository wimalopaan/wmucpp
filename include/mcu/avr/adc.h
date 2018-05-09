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
#include "mcu/avr8.h"
#include "mcu/avr/groups.h"
//#include "mcu/ports.h"
#include "hal/event.h"
#include "util/types.h"
#include "util/dassert.h"
#include "util/fixedpoint.h"
#include "util/rational.h"

namespace AVR {
   
    template<uint8_t T>
    struct Resolution;
    
    template<>
    struct Resolution<10> {
        typedef uint_ranged<uint16_t, 0, 1023> type;
    };
    template<>
    struct Resolution<8> {
        typedef uint_ranged<uint8_t, 0, 255> type;
    };
    
    template<uint8_t N, typename Reso = Resolution<10>, typename VREF = AD::VRef<AD::V1_1, DefaultMcuType>, typename MCU = DefaultMcuType>
    class Adc final {
        static_assert(N < MCU::Adc::count, "wrong adc number"); 
        
    public:
        static constexpr auto mcuAdc = getBaseAddr<typename MCU::Adc, N>;
        static constexpr auto channelMask = MCU::Adc::MUX::mux3 | MCU::Adc::MUX::mux2 | MCU::Adc::MUX::mux1 | MCU::Adc::MUX::mux0;
        
        typedef typename Reso::type value_type;
        typedef FixedPoint<uint16_t, 8> voltage_type;
        
//        static constexpr auto VRef = MCU::Adc::template Parameter<N>::VRef;
        static constexpr auto VRef = VREF::value;
        static constexpr auto refs = VREF::refs;
        
        static constexpr double VBit = VRef / Reso::type::Upper;
        
        typedef MCU mcu_type;
        
        static constexpr const uint8_t number = N;
        typedef typename MCU::Adc::template Parameter<N> mcuadc_parameter_type;
        
        Adc() = delete;
        
        static void init() {
            if constexpr(std::is_same<Reso, Resolution<8>>::value) {
                mcuAdc()->admux.template add<refs | MCU::Adc::MUX::adlar, DisbaleInterrupt<NoDisableEnable>>();
            }
            else {
                mcuAdc()->admux.template add<refs, DisbaleInterrupt<NoDisableEnable>>();
            }
            mcuAdc()->adcsra.template add<MCU::Adc::SRA::aden | MCU::Adc::SRA::adps2 | MCU::Adc::SRA::adps1 | MCU::Adc::SRA::adps0, DisbaleInterrupt<NoDisableEnable>>();
        }
        
        static void startConversion() {
            assert(conversionReady());
            mcuAdc()->adcsra.template add<MCU::Adc::SRA::adsc, DisbaleInterrupt<NoDisableEnable>>();
        }
        
        static bool conversionReady() {
            return !mcuAdc()->adcsra.template isSet<MCU::Adc::SRA::adsc>();
        }
        
        static typename Reso::type value() {
            if constexpr(std::is_same<Reso, Resolution<8>>::value) {
                return typename Reso::type{*mcuAdc()->reg.adch}; // adch
            }
            else {
                return typename Reso::type{*mcuAdc()->adc};
            }
        }
        
        static FixedPoint<uint16_t, 8> toVoltage(uint16_t v) {
            return FixedPoint<uint16_t, 8>{v * VBit};
        }
        
        static void channel(uint8_t ch) {
            assert(ch < mcuadc_parameter_type::channelMasks.size);
            mcuAdc()->admux.template setPartial<channelMask, DisbaleInterrupt<NoDisableEnable>>(mcuadc_parameter_type::channelMasks[ch]);
        }
    };
    
}
