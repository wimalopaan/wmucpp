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
#include "mcu/ports.h"

namespace AVR {
    template<uint8_t N, typename MCU = DefaultMcuType>
    struct AdCompParameter;
    
    template<>
    struct AdCompParameter<0, ATMega1284P> {
        AdCompParameter() = delete;
        using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
        using ain0 = AVR::Pin<PortB, 2>;
        using ain1 = AVR::Pin<PortB, 3>;
    };
    
    // todo: Liste der Pins/Channels
    template<uint8_t N, typename MCU = DefaultMcuType>
    class AdComparator final {
        static_assert(N < MCU::AdComparator::count, "wrong adcomparator number"); 
        AdComparator() = delete;
        using parameter = AdCompParameter<N, MCU>;

        typedef typename MCU::Adc::template Parameter<N> mcuadc_parameter_type;
        static constexpr auto channelMask = MCU::Adc::MUX::mux3 | MCU::Adc::MUX::mux2 | MCU::Adc::MUX::mux1 | MCU::Adc::MUX::mux0;
    public:
        enum class Mode : uint8_t {OnToggle, OnRising, OnFalling};
        
        static constexpr auto mcuAdComparator = getBaseAddr<typename MCU::AdComparator, N>;
        static constexpr auto mcuAdc = getBaseAddr<typename MCU::Adc, N>;
        
        static void init() {
            mcuAdc()->adcsra.template clear<MCU::Adc::SRA::aden>();
            mcuAdc()->adcsrb.template set<MCU::Adc::SRB::acme>();
//            mcuAdComparator()->acsr =  _BV(ACIE) | _BV(ACI) | _BV(ACIS1) | _BV(ACIS0);
        }
        // todo: ranged type (Anzahl dre Channels)
        static void channel(uint8_t ch) {
            assert(ch < mcuadc_parameter_type::channelMasks.size);
            mcuAdc()->admux.template setPartial<channelMask, DisbaleInterrupt<NoDisableEnable>>(mcuadc_parameter_type::channelMasks[ch]);
        }
        
        static void set(Mode mode) {
            constexpr auto modeMask = MCU::AdComparator::SR::acis1 | MCU::AdComparator::SR::acis0;
            if (mode == Mode::OnToggle) {
//                mcuAdComparator()->acsr.template setPartial<modeMask, DisbaleInterrupt<NoDisableEnable>>(0);
            }
            else if (mode == Mode::OnRising) {
                mcuAdComparator()->acsr.template setPartial<modeMask, DisbaleInterrupt<NoDisableEnable>>(MCU::AdComparator::SR::acis1 | MCU::AdComparator::SR::acis0);
            }
            else if (mode == Mode::OnFalling) {
                mcuAdComparator()->acsr.template setPartial<modeMask, DisbaleInterrupt<NoDisableEnable>>(MCU::AdComparator::SR::acis1);
            }
        }
        static bool get() {
            bool v = mcuAdComparator()->acsr.template isSet<MCU::AdComparator::SR::aci>();
            mcuAdComparator()->acsr.template add<MCU::AdComparator::SR::aci>();
            return v;
        }
        static bool getO() {
            return mcuAdComparator()->acsr.template isSet<MCU::AdComparator::SR::aco>();
        }
    private:
    };
    
}
