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
#include "../common/groups.h"

namespace AVR {
    template<uint8_t N, typename MCU = DefaultMcuType>
    struct AdCompParameter;
    
    template<>
    struct AdCompParameter<0, ATMega1284P> {
        AdCompParameter() = delete;
        using PortB = AVR::Port<AVR::B>;
        using ain0 = AVR::Pin<PortB, 2>;
        using ain1 = AVR::Pin<PortB, 3>;
    };
    
    template<typename ComponentNumber, typename MCU = DefaultMcuType> struct AdComparator;
    
    template<AVR::Concepts::ComponentNumber CN, AVR::Concepts::At01Series MCU>
    struct AdComparator<CN, MCU> final {
        inline static constexpr uint8_t number = CN::value;
        
        inline static constexpr auto mcu_comp  = AVR::getBaseAddr<typename MCU::AdComparator, number>;
        
        using ctrla_t = typename MCU::AdComparator::CtrlA1_t;
        using edge_t = typename MCU::AdComparator::CtrlA2_t;
        using hyst_t = typename MCU::AdComparator::CtrlA3_t;
        using status_t = typename MCU::AdComparator::Status_t;
        using intctrl_t = typename MCU::AdComparator::IntCtrl_t;

        using mux_p_t = typename MCU::AdComparator::MuxCtrl2_t;
        using mux_n_t = typename MCU::AdComparator::MuxCtrl3_t;
        
        using index_type = etl::uint_ranged<uint8_t, 0, 3>;
        
        inline static void init() {
            mcu_comp()->ctrla.template set<hyst_t::hyst_large>();
            mcu_comp()->ctrla.template add<ctrla_t::outen>();
            mcu_comp()->ctrla.template add<ctrla_t::enable>();
        }
        
        inline static void hysterese(hyst_t v) {
            mcu_comp()->ctrla.setPartial(v);
        }
        
        template<bool B>
        inline static void enableInterrupts() {
            if constexpr(B) {
                mcu_comp()->status.template reset<status_t::cmp>();
                mcu_comp()->intctrl.template set<intctrl_t::cmp>();
            }
            else {
                mcu_comp()->intctrl.template clear<intctrl_t::cmp>();
            }
        }

        inline static bool get() {
            return mcu_comp()->status.template isSet<status_t::state>();    
        }
        
        inline static void onInterrupt(auto f) {
            f();
            mcu_comp()->status.template reset<status_t::cmp>();
        }
        
        struct Positiv : std::integral_constant<edge_t, edge_t::positive_edge> {};
        struct Negativ : std::integral_constant<edge_t, edge_t::negative_edge> {};
        
        inline static void onEdge(auto f) {
            mcu_comp()->status.template testAndReset<status_t::cmp>([&](){
                f();
            });
        }
        
        template<typename T, typename IMode = etl::RestoreState>
        inline static void edge() {
            mcu_comp()->ctrla.template setPartial<edge_t, etl::DisbaleInterrupt<IMode>>(T::value);
        }
        
        template<typename IMode = etl::RestoreState>
        inline static void positiv_channel(index_type n) {
            mcu_comp()->muxctrl.template setPartial<mux_p_t, etl::DisbaleInterrupt<IMode>>(mux_p_t(n.toInt() << register_bit_position_v<mux_p_t>));
        }
        template<typename IMode = etl::RestoreState>
        inline static void negativ_channel(index_type n) {
            mcu_comp()->muxctrl.template setPartial<mux_n_t, etl::DisbaleInterrupt<IMode>>(mux_n_t(n.toInt() << register_bit_position_v<mux_n_t>));
        }
    };
    
    // todo: Liste der Pins/Channels
    template<typename CN, AVR::Concepts::AtMega MCU>
    class AdComparator<CN, MCU> final {
        inline static constexpr uint8_t N = CN::value;
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
        static void enable() {
            mcuAdc()->adcsrb.template add<MCU::Adc::SRB::acme>();
        }
        static void disable() {
            mcuAdc()->adcsrb.template clear<MCU::Adc::SRB::acme>();
        }
        // todo: ranged type (Anzahl dre Channels)
        inline static void channel(uint8_t ch) {
            assert(ch < mcuadc_parameter_type::channelMasks.size);
            mcuAdc()->admux.template setPartial<channelMask, etl::DisbaleInterrupt<etl::NoDisableEnable>>(mcuadc_parameter_type::channelMasks[ch]);
        }
        inline static void reset() {
            mcuAdComparator()->acsr.template add<MCU::AdComparator::SR::aci>();
        }
        
        inline static void enableCapture(bool on = true) {
            if (on) {
                mcuAdComparator()->acsr.template add<MCU::AdComparator::SR::acic>();
            }
            else {
                mcuAdComparator()->acsr.template clear<MCU::AdComparator::SR::acic>();
            }
        }
        
        inline static void set(Mode mode) {
            constexpr auto modeMask = MCU::AdComparator::SR::acis1 | MCU::AdComparator::SR::acis0;
            if (mode == Mode::OnToggle) {
//                mcuAdComparator()->acsr.template setPartial<modeMask, DisbaleInterrupt<NoDisableEnable>>(0);
            }
            else if (mode == Mode::OnRising) {
                mcuAdComparator()->acsr.template setPartial<modeMask, etl::DisbaleInterrupt<etl::NoDisableEnable>>(MCU::AdComparator::SR::acis1 | MCU::AdComparator::SR::acis0);
            }
            else if (mode == Mode::OnFalling) {
                mcuAdComparator()->acsr.template setPartial<modeMask, etl::DisbaleInterrupt<etl::NoDisableEnable>>(MCU::AdComparator::SR::acis1);
            }
        }
        inline static bool get() {
            bool v = mcuAdComparator()->acsr.template isSet<MCU::AdComparator::SR::aci>();
            mcuAdComparator()->acsr.template add<MCU::AdComparator::SR::aci>();
            return v;
        }
        inline static bool get1() {
            return mcuAdComparator()->acsr.template isSet<MCU::AdComparator::SR::aci>();
        }
        inline static bool getO() {
            return mcuAdComparator()->acsr.template isSet<MCU::AdComparator::SR::aco>();
        }
    private:
    };
    
}
