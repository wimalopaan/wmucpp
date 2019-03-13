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
#include <etl/rational.h>
#include <etl/fixedpoint.h>

#include "groups.h"

namespace AVR {
    namespace AD {
        
        struct V1_1;
        struct V2_56;
        
        template<uint16_t Volts, uint16_t MilliVolts>
        struct Vextern {
            static constexpr float value = Volts + (0.001f * MilliVolts);
        };
        template<typename Voltage, typename MCU> 
        struct VRef {
            static constexpr auto refs = typename MCU::Adc::MUX{0};
            static constexpr float value = Voltage::value;
        };
    }
    
    struct HighSpeed;
    struct LowSpeed;
    
    template<uint8_t T>
    struct Resolution;
    
    template<>
    struct Resolution<10> {
        typedef etl::uint_ranged<uint16_t, 0, 1023> type;
    };
    template<>
    struct Resolution<8> {
        typedef etl::uint_ranged<uint8_t, 0, 255> type;
    };
    
    template<uint8_t N, typename Reso = Resolution<10>, typename VREF = AD::VRef<AD::V1_1, DefaultMcuType>, typename MCU = DefaultMcuType>
    class Adc final {
        static_assert(N < MCU::Adc::count, "wrong adc number"); 
        
    public:
        static constexpr auto mcuAdc = getBaseAddr<typename MCU::Adc, N>;
        static constexpr auto channelMask = MCU::Adc::MUX::mux3 | MCU::Adc::MUX::mux2 | MCU::Adc::MUX::mux1 | MCU::Adc::MUX::mux0;
        
        typedef typename Reso::type value_type;
        typedef etl::FixedPoint<uint16_t, 8> voltage_type;
        
//        static constexpr auto VRef = MCU::Adc::template Parameter<N>::VRef;
        static constexpr auto VRef = VREF::value;
        static constexpr auto refs = VREF::refs;
        
        static constexpr double VBit = VRef / Reso::type::Upper;
        
        typedef MCU mcu_type;
        
        static constexpr const uint8_t number = N;
        typedef typename MCU::Adc::template Parameter<N> mcuadc_parameter_type;
        
        Adc() = delete;
        
        template<typename S = LowSpeed>
        static void init() {
            if constexpr(std::is_same_v<Reso, Resolution<8>>) {
//                mcuAdc()->admux.template add<refs | MCU::Adc::MUX::adlar, DisbaleInterrupt<NoDisableEnable>>();
                mcuAdc()->admux.template set<refs | MCU::Adc::MUX::adlar>();
            }
            else {
                mcuAdc()->admux.template add<refs, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            }
            if constexpr(std::is_same_v<S, HighSpeed>) {
                mcuAdc()->adcsra.template add<MCU::Adc::SRA::aden | MCU::Adc::SRA::adps1 | MCU::Adc::SRA::adps0, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            }
            else {
                mcuAdc()->adcsra.template add<MCU::Adc::SRA::aden | MCU::Adc::SRA::adps2 | MCU::Adc::SRA::adps1 | MCU::Adc::SRA::adps0, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            }
        }
        
        inline static void enable() {
            mcuAdc()->adcsra.template add<MCU::Adc::SRA::aden>();
        }
        inline static void disable() {
            mcuAdc()->adcsra.template clear<MCU::Adc::SRA::aden>();
        }
        
        static void startConversion() {
            assert(conversionReady());
            mcuAdc()->adcsra.template clear<MCU::Adc::SRA::adif, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            mcuAdc()->adcsra.template add<MCU::Adc::SRA::adsc, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
        }
        
        static bool conversionReady() {
            return !mcuAdc()->adcsra.template isSet<MCU::Adc::SRA::adsc>();
        }

        template<typename F>
        static void whenConversionReady(const F& f) {
            if (mcuAdc()->adcsra.template isSet<MCU::Adc::SRA::adif>()) {
                f(value());
            }
        }

        template<typename F>
        static void waitUntilConversionReady(const F& f) {
            while(!mcuAdc()->adcsra.template isSet<MCU::Adc::SRA::adif>());
            f(value());
        }
        
        static typename Reso::type value() {
            if constexpr(std::is_same<Reso, Resolution<8>>::value) {
                return typename Reso::type{*mcuAdc()->reg.adch}; // adch
            }
            else {
                return typename Reso::type{*mcuAdc()->adc};
            }
        }
        
        static etl::FixedPoint<uint16_t, 8> toVoltage(uint16_t v) {
            return etl::FixedPoint<uint16_t, 8>{v * VBit};
        }
        
        static void channel(uint8_t ch) {
            assert(ch < mcuadc_parameter_type::channelMasks.size);
            mcuAdc()->admux.template setPartial<channelMask, etl::DisbaleInterrupt<etl::NoDisableEnable>>(mcuadc_parameter_type::channelMasks[ch]);
        }
    };
    
}
