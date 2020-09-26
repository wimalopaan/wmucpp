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

#include "../common/groups.h"

namespace AVR {
    namespace AD {
        struct V1_1 {
            static constexpr double value = 1.1;
        };
        struct V2_56 {
            static constexpr double value = 2.56;
        };
        
        template<uint16_t Volts, uint16_t MilliVolts>
        struct Vextern {
            static constexpr double value = Volts + (0.001 * MilliVolts);
        };
        
        template<typename Voltage, typename MCU = DefaultMcuType> 
        struct VRef {
            static constexpr auto refs = typename MCU::Adc::MUX{0};
            static constexpr double value = Voltage::value;
        };
    }
    namespace Vref {
        struct V0_55 {
            inline static constexpr double value = 0.55;
        };
        struct V1_1 {
            inline static constexpr double value = 1.1;
        };
        struct V1_5 {
            inline static constexpr double value = 1.5;
        };
        struct V2_5 {
            inline static constexpr double value = 2.5;
        };
        struct V4_3 {
            inline static constexpr double value = 4.3;
        };
        namespace detail {
            template<typename V> struct isVref : std::false_type {};
            template<> struct isVref<V0_55> : std::true_type {};
            template<> struct isVref<V1_1> : std::true_type {};
            template<> struct isVref<V1_5> : std::true_type {};
            template<> struct isVref<V2_5> : std::true_type {};
            template<> struct isVref<V4_3> : std::true_type {};
        }
    }
    
    struct HighSpeed;
    struct LowSpeed;
    
    template<uint8_t T>
    struct Resolution;
    
    template<>
    struct Resolution<10> {
        typedef etl::uint_ranged<uint16_t, 0, 1023> type;
        inline static constexpr uint8_t bits = 10;
    };
    template<>
    struct Resolution<8> {
        typedef etl::uint_ranged<uint8_t, 0, 255> type;
        inline static constexpr uint8_t bits = 8;
    };

    
    
    
    template<AVR::Concepts::ComponentNumber CN, typename Reso = Resolution<10>, typename VRefType = AD::VRef<AD::V1_1, DefaultMcuType>, typename MCU = DefaultMcuType>
    class Adc;

    template<AVR::Concepts::ComponentNumber CN, typename Reso, typename VRefType, AVR::Concepts::At01Series MCU>
    requires ((CN::value <= 1) && (Vref::detail::isVref<VRefType>::value))
    class Adc<CN, Reso, VRefType, MCU> final {
        inline static constexpr uint8_t number = CN::value;
        static inline constexpr auto mcu_adc  = AVR::getBaseAddr<typename MCU::Adc, number>;
        static inline constexpr auto mcu_vref = AVR::getBaseAddr<typename MCU::Vref>;
        
        using ca_t = typename MCU::Adc::CtrlA_t;
        using cb_t = typename MCU::Adc::CtrlB_t;
        using cc_t = typename MCU::Adc::CtrlC_t;
        using cd_t = typename MCU::Adc::CtrlD_t;
        using co_t = typename MCU::Adc::Command_t;
        using int_t = typename MCU::Adc::IntCtrl_t;
        
        using va1_t = typename MCU::Vref::CtrlA1_t;
        using va2_t = typename MCU::Vref::CtrlA2_t;
        using vb_t = typename MCU::Vref::CtrlB_t;
    public:
        using reso_type = Reso;
        using value_type = typename Reso::type;

        static constexpr auto VRef = VRefType::value;
//        static constexpr auto refs = VRefType::refs;
        
        static constexpr double VBit = VRef / Reso::type::Upper;

        template<typename V>
        requires (Vref::detail::isVref<V>::value)
        inline static void set() {
            if constexpr(AVR::Groups::isAtMega0<MCU>::value) {
                if constexpr(std::is_same_v<V, Vref::V1_1>) {
                    mcu_vref()->ctrla.template set<va1_t::adc_V1_1>();           
                }
                else if constexpr(std::is_same_v<V, Vref::V1_5>) {
                    mcu_vref()->ctrla.template set<va1_t::adc_V1_5>();           
                }
                else if constexpr(std::is_same_v<V, Vref::V2_5>) {
                    mcu_vref()->ctrla.template set<va1_t::adc_V2_5>();           
                }
                else if constexpr(std::is_same_v<V, Vref::V4_3>) {
                    mcu_vref()->ctrla.template set<va1_t::adc_V4_3>();           
                } 
                else {
                    static_assert(std::false_v<V>, "wrong VRef selection");
                }
                mcu_vref()->ctrlb.template set<vb_t::adc_refen>();           
            }
            else if constexpr(AVR::Groups::isAtTiny1<MCU>::value) {
                if constexpr(CN::value == 0) {
                    if constexpr(std::is_same_v<V, Vref::V1_1>) {
                        mcu_vref()->ctrla.template setPartial(va1_t::adc0_V1_1);           
                    }
                    else if constexpr(std::is_same_v<V, Vref::V1_5>) {
                        mcu_vref()->ctrla.template setPartial(va1_t::adc0_V1_5);           
                    }
                    else if constexpr(std::is_same_v<V, Vref::V2_5>) {
                        mcu_vref()->ctrla.template setPartial(va1_t::adc0_V2_5);           
                    }
                    else if constexpr(std::is_same_v<V, Vref::V4_3>) {
                        mcu_vref()->ctrla.template setPartial(va1_t::adc0_V4_3);           
                    } 
                    else {
                        static_assert(std::false_v<V>, "wrong VRef selection");
                    }
                    mcu_vref()->ctrlb.template add<vb_t::adc0_refen>();           
                }
                else if constexpr(CN::value == 1) {
                    using vc1_t = typename MCU::Vref::CtrlC1_t; // only attiny1
                    if constexpr(std::is_same_v<V, Vref::V0_55>) {
                        mcu_vref()->ctrlc.template setPartial(vc1_t::adc1_V0_55);           
                    }
                    else if constexpr(std::is_same_v<V, Vref::V1_1>) {
                        mcu_vref()->ctrlc.template setPartial(vc1_t::adc1_V1_1);           
                    }
                    else if constexpr(std::is_same_v<V, Vref::V1_5>) {
                        mcu_vref()->ctrlc.template setPartial(vc1_t::adc1_V1_5);           
                    }
                    else if constexpr(std::is_same_v<V, Vref::V2_5>) {
                        mcu_vref()->ctrlc.template setPartial(vc1_t::adc1_V2_5);           
                    }
                    else if constexpr(std::is_same_v<V, Vref::V4_3>) {
                        mcu_vref()->ctrlc.template add<vc1_t::adc1_V4_3>();           
                    } 
                    else {
                        static_assert(std::false_v<V>, "wrong VRef selection");
                    }
                    mcu_vref()->ctrlb.template add<vb_t::adc1_refen>();           
                }
                else {
                    static_assert(std::false_v<V>);
                }
            }
            else {
                static_assert(std::false_v<V>);
            }
        }
        
        template<bool On>
        inline static void enable() {
            if constexpr(On) {
                if constexpr(std::is_same_v<Reso, Resolution<8>>) {
                    mcu_adc()->ctrla.template set<ca_t::ressel | ca_t::enable>();           
                }
                else if constexpr(std::is_same_v<Reso, Resolution<10>>) {
                    mcu_adc()->ctrla.template set<ca_t::enable>();           
                }
                else {
                    static_assert(std::false_v<Reso>, "wrong resolution");
                }
            }
            else {
                    mcu_adc()->ctrla.template clear<ca_t::enable>();           
            }
        }
        
        inline static void init() {
            set<VRefType>();

            if constexpr(CN::value == 0) {
                mcu_adc()->ctrlc.template set<cc_t::samcap |cc_t::div256 | cc_t::ref_internal>();           
            }
            if constexpr(CN::value == 1) {
                mcu_adc()->ctrlc.template set<cc_t::samcap | cc_t::div256 | cc_t::ref_internal>();           
            }

            enable<true>();            
        }

        inline static void startConversion() {
            mcu_adc()->command.template set<co_t::stconv>();           
        }
        
        template<typename F>
        inline static void whenConversionReady(const F& f) {
            mcu_adc()->intflags.template testAndReset<int_t::resrdy>([&]{
                f();
            });
        }

        inline static void nsamples(const uint8_t n) {
            switch(n) {
            case 0x01:  
                mcu_adc()->ctrlb.template set<cb_t::accu_2>();           
                break;
            case 0x02:  
                mcu_adc()->ctrlb.template set<cb_t::accu_4>();           
                break;
            case 0x03:  
                mcu_adc()->ctrlb.template set<cb_t::accu_8>();           
                break;
            case 0x04:  
                mcu_adc()->ctrlb.template set<cb_t::accu_16>();           
                break;
            case 0x05:  
                mcu_adc()->ctrlb.template set<cb_t::accu_32>();           
                break;
            case 0x06:  
                mcu_adc()->ctrlb.template set<cb_t::accu_64>();           
                break;
            default:
                mNSamples = 1;
                mcu_adc()->ctrlb.template set<cb_t::accu_none>();           
                break;
            }
        }
        
        inline static auto value() {
            return value_type{*mcu_adc()->res};
        }

        inline static auto valueShifted() {
            return value_type{(*mcu_adc()->res) >> mNSamples};
        }
        
        static void channel(uint8_t ch) {
            if (ch == 0x1e) { // temperature
                enable<false>();
                *mcu_adc()->sampctl = std::byte{31};
                mcu_adc()->ctrld.template set<cd_t::delay256>();           
                set<Vref::V1_1>();
                enable<true>();
            }
            else {
                if (mcu_adc()->muxpos.value() == typename MCU::Adc::MuxPos_t{0x1e}) {
                    enable<false>();
                    *mcu_adc()->sampctl = std::byte{0};
                    mcu_adc()->ctrld.template set<cd_t::delay0>();           
                    set<VRefType>();
                    enable<true>();
                }
            }
            mcu_adc()->muxpos.template set(typename MCU::Adc::MuxPos_t{ch});
        }
        
        private:

        inline static bool conversionReady() {
            return mcu_adc()->intflags.template isSet<int_t::resrdy>();        
        }
        
        inline static uint8_t mNSamples{1};
    };
    
    
    template<AVR::Concepts::ComponentNumber CN, typename Reso, typename VRefType, AVR::Concepts::AtMega MCU>
    class Adc<CN, Reso, VRefType, MCU> final {
        static_assert(CN::value < MCU::Adc::count, "wrong adc number"); 
        
        static inline constexpr auto N = CN::value;
    public:
        static constexpr auto mcuAdc = getBaseAddr<typename MCU::Adc, N>;
        static constexpr auto channelMask = MCU::Adc::MUX::mux3 | MCU::Adc::MUX::mux2 | MCU::Adc::MUX::mux1 | MCU::Adc::MUX::mux0;
        
        using reso_type = Reso;
        typedef typename Reso::type value_type;
        typedef etl::FixedPoint<uint16_t, 8> voltage_type;
        
//        static constexpr auto VRef = MCU::Adc::template Parameter<N>::VRef;
        static constexpr auto VRef = VRefType::value;
        static constexpr auto refs = VRefType::refs;
        
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
            assert(ch < mcuadc_parameter_type::channelMasks.size());
            
            mcuAdc()->admux.template setPartial<channelMask, etl::DisbaleInterrupt<etl::NoDisableEnable>>(mcuadc_parameter_type::channelMasks[ch]);
        }
        
//        std::integral_constant<uint8_t, (uint8_t)mcuadc_parameter_type::channelMasks[6]>::_;;
//        std::integral_constant<uint8_t, (uint8_t)mcuadc_parameter_type::channelMasks[7]>::_;;
        
    };
    
}
