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

#include <etl/meta.h>

#include "../common/concepts.h"
#include "../common/groups.h"

namespace AVR {
    namespace Portmux {
        namespace detail {
            namespace spi {
                template<typename T, typename MCU = DefaultMcuType>
                struct Mapper {
                    using type = void;    
                };

                template<AVR::Concepts::AtTiny1 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Spi<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CtrlB_t;
                    using type = std::integral_constant<route_t, route_t::spi0_alt1>;
                };
            }
            namespace twi {
                template<typename T, typename MCU = DefaultMcuType>
                struct Mapper {
                    using type = void;    
                };

                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Twi<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TwiRoute_t;
                    using type = std::integral_constant<route_t, route_t::twi0_alt1>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Twi<0>, AVR::Portmux::Alt2>, MCU> {
                    using route_t = typename MCU::Portmux::TwiRoute_t;
                    using type = std::integral_constant<route_t, route_t::twi0_alt2>;
                };
            }
            namespace usart {
                template<typename T, typename MCU = DefaultMcuType>
                struct Mapper {
                    using type = void;    
                };

                template<AVR::Concepts::AtTiny1 MCU, typename P>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<0>, P>, MCU> {
                    static_assert(std::false_v<P>, "wrong position");
                };
                template<AVR::Concepts::AtTiny1 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Default>, MCU> {
                    using type = void;
                };
                template<AVR::Concepts::AtTiny1 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CtrlB_t;
                    using type = std::integral_constant<route_t, route_t::usart0_alt1>;
                };

                
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart0_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart1_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart2_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart3_alt1>;
                };

                template<AVR::Concepts::AtDaSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart0_alt1>;
                };
                template<AVR::Concepts::AtDaSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart1_alt1>;
                };
                template<AVR::Concepts::AtDaSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart2_alt1>;
                };
                template<AVR::Concepts::AtDaSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRoute_t;
                    using type = std::integral_constant<route_t, route_t::usart3_alt1>;
                };



                template<AVR::Concepts::AtDbSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRouteA_t;
                    using type = std::integral_constant<route_t, route_t::usart0_alt1>;
                };
                template<AVR::Concepts::AtDbSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<1>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRouteA_t;
                    using type = std::integral_constant<route_t, route_t::usart1_alt1>;
                };
                template<AVR::Concepts::AtDbSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<2>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRouteA_t;
                    using type = std::integral_constant<route_t, route_t::usart2_alt1>;
                };
                template<AVR::Concepts::AtDbSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Usart<3>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::UsartRouteA_t;
                    using type = std::integral_constant<route_t, route_t::usart3_alt1>;
                };

                
            }
            namespace ccl {
                template<typename T, typename MCU = DefaultMcuType>
                struct Mapper {
                    using type = void;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Ccl<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CclRoute_t;
                    using type = std::integral_constant<route_t, route_t::lut0_alt1>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Ccl<1>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CclRoute_t;
                    using type = std::integral_constant<route_t, route_t::lut1_alt1>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CclRoute_t;
                    using type = std::integral_constant<route_t, route_t::lut2_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Ccl<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CclRoute_t;
                    using type = std::integral_constant<route_t, route_t::lut0_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Ccl<1>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CclRoute_t;
                    using type = std::integral_constant<route_t, route_t::lut1_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Ccl<2>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CclRoute_t;
                    using type = std::integral_constant<route_t, route_t::lut2_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Ccl<3>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::CclRoute_t;
                    using type = std::integral_constant<route_t, route_t::lut3_alt1>;
                };
            }
            namespace tca {
                template<typename T, typename MCU = DefaultMcuType>
                struct Mapper {
                    using type = void;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltA>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onA>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltB>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onB>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltC>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onC>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltD>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onD>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltE>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onE>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltF>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onF>;
                };
                
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltA>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onA>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltB>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onB>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltC>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onC>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltD>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onD>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltE>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onE>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltF>, MCU> {
                    using route_t = typename MCU::Portmux::TcaRoute_t;
                    using type = std::integral_constant<route_t, route_t::onF>;
                };
                
                
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltA>, MCU> {
                    using route_t = typename MCU::Portmux::Tca0Route_t;
                    using type = std::integral_constant<route_t, route_t::onA>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltB>, MCU> {
                    using route_t = typename MCU::Portmux::Tca0Route_t;
                    using type = std::integral_constant<route_t, route_t::onB>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltC>, MCU> {
                    using route_t = typename MCU::Portmux::Tca0Route_t;
                    using type = std::integral_constant<route_t, route_t::onC>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltD>, MCU> {
                    using route_t = typename MCU::Portmux::Tca0Route_t;
                    using type = std::integral_constant<route_t, route_t::onD>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltE>, MCU> {
                    using route_t = typename MCU::Portmux::Tca0Route_t;
                    using type = std::integral_constant<route_t, route_t::onE>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<0>, AVR::Portmux::AltF>, MCU> {
                    using route_t = typename MCU::Portmux::Tca0Route_t;
                    using type = std::integral_constant<route_t, route_t::onF>;
                };
                
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<1>, AVR::Portmux::AltB>, MCU> {
                    using route_t = typename MCU::Portmux::Tca1Route_t;
                    using type = std::integral_constant<route_t, route_t::onB>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<1>, AVR::Portmux::AltC>, MCU> {
                    using route_t = typename MCU::Portmux::Tca1Route_t;
                    using type = std::integral_constant<route_t, route_t::onC>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<1>, AVR::Portmux::AltE>, MCU> {
                    using route_t = typename MCU::Portmux::Tca1Route_t;
                    using type = std::integral_constant<route_t, route_t::onE>;
                };
                template<AVR::Concepts::AtDx64Series MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tca<1>, AVR::Portmux::AltG>, MCU> {
                    using route_t = typename MCU::Portmux::Tca1Route_t;
                    using type = std::integral_constant<route_t, route_t::onG>;
                };
                
            }
            namespace tcb {
                template<typename T, typename MCU = DefaultMcuType>
                struct Mapper {
                    using type = void;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb0_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<1>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb1_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<2>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb2_alt1>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<3>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb3_alt1>;
                };

                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<0>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb0_alt1>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<1>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb1_alt1>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<2>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb2_alt1>;
                };
                template<AVR::Concepts::AtDxSeries MCU>
                struct Mapper<AVR::Portmux::Position<AVR::Component::Tcb<3>, AVR::Portmux::Alt1>, MCU> {
                    using route_t = typename MCU::Portmux::TcbRoute_t;
                    using type = std::integral_constant<route_t, route_t::tcb3_alt1>;
                };

                
            }
        }
        template<typename CCList, typename MCU = DefaultMcuType>
        struct StaticMapper;

        template<typename CCList, AVR::Concepts::AtDxSeriesAll MCU>
        struct StaticMapper<CCList, MCU> final {
        private:
            static constexpr auto mcu_pmux = getBaseAddr<typename MCU::Portmux>;
            using usart_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::usart::Mapper, CCList>>;
            using ccl_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::ccl::Mapper, CCList>>;
            using tca_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::tca::Mapper, CCList>>;
            using tcb_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::tcb::Mapper, CCList>>;
            using twi_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::twi::Mapper, CCList>>;
            
            // sizes / counts of components must be adapted to CPU types
            static_assert(Meta::size_v<usart_list> <= 4);
            static_assert(Meta::size_v<ccl_list> <= 4);
            static_assert(Meta::size_v<tca_list> <= 2);
            static_assert(Meta::size_v<ccl_list> <= 4);
            static_assert(Meta::size_v<twi_list> <= 2);
            
        public:
            static inline void init() {
//                static_assert(std::false_v<MCU>);
                if constexpr(Meta::size_v<usart_list> > 0) {
                    constexpr auto value = Meta::value_or_v<usart_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    // Unterscheidung fuer usart >4 fehlt noch
                    mcu_pmux()->usartroutea.template set<value>();                
                }
                if constexpr(Meta::size_v<ccl_list> > 0) {
                    constexpr auto value = Meta::value_or_v<ccl_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->cclroutea.template set<value>();                
                }
                
                // allow bitmask on different types
                if constexpr(Meta::size_v<tca_list> > 0) {
//                    constexpr auto value = Meta::front<tca_list>::value;
                    constexpr auto value = Meta::nth_element<0, tca_list>::value;
//                    constexpr auto value = Meta::value_or_v<tca_list>;
                    mcu_pmux()->tcaroutea.template set<value>();
                    if constexpr(Meta::size_v<tca_list> == 2) {
                        constexpr auto value = Meta::nth_element<1, tca_list>::value;
                        mcu_pmux()->tcaroutea.template add<value>();
                    }
                }
                if constexpr(Meta::size_v<tcb_list> > 0) {
                    constexpr auto value = Meta::value_or_v<tcb_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->tcbroutea.template set<value>();                
                }
                if constexpr(Meta::size_v<twi_list> > 0) {
                    constexpr auto value = Meta::value_or_v<twi_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->twiroutea.template set<value>();                
                }
            }
        };
        
        template<typename CCList, AVR::Concepts::AtMega0 MCU>
        struct StaticMapper<CCList, MCU> final {
        private:
            static constexpr auto mcu_pmux = getBaseAddr<typename MCU::Portmux>;
            using usart_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::usart::Mapper, CCList>>;
            using ccl_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::ccl::Mapper, CCList>>;
            using tca_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::tca::Mapper, CCList>>;
            using tcb_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::tcb::Mapper, CCList>>;
            
            static_assert(Meta::size_v<usart_list> <= 4);
            static_assert(Meta::size_v<ccl_list> <= 4);
            static_assert(Meta::size_v<tca_list> <= 1);
            
        public:
            static inline void init() {
                if constexpr(Meta::size_v<usart_list> > 0) {
                    constexpr auto value = Meta::value_or_v<usart_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->usartroutea.template set<value>();                
                }
                if constexpr(Meta::size_v<ccl_list> > 0) {
                    constexpr auto value = Meta::value_or_v<ccl_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->cclroutea.template set<value>();                
                }
                if constexpr(Meta::size_v<tca_list> > 0) {
                    constexpr auto value = Meta::front<tca_list>::value;
//                    tca_list::_;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->tcaroutea.template set<value>();                
                }
                if constexpr(Meta::size_v<tcb_list> > 0) {
                    constexpr auto value = Meta::value_or_v<tcb_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->tcbroutea.template set<value>();                
                }
            }
        };

        template<typename CCList, AVR::Concepts::AtTiny1 MCU>
        struct StaticMapper<CCList, MCU> final {
        private:
            static constexpr auto mcu_pmux = getBaseAddr<typename MCU::Portmux>;
            using usart_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::usart::Mapper, CCList>>;
            using tca_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::tca::Mapper, CCList>>;
            using spi_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::spi::Mapper, CCList>>;
            
            static_assert(Meta::size_v<usart_list> <= 4);
            static_assert(Meta::size_v<tca_list> <= 1);
            static_assert(Meta::size_v<spi_list> <= 1);
            
        public:
            static inline void init() {
                if constexpr(Meta::size_v<usart_list> > 0) {
                    constexpr auto value = Meta::value_or_v<usart_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->ctrlb.template set<value>();                
                }
                if constexpr(Meta::size_v<tca_list> > 0) {
                    constexpr auto value = Meta::front<tca_list>::value;
//                    tca_list::_;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->ctrlc.template set<value>();                
                }
                if constexpr(Meta::size_v<spi_list> > 0) {
                    constexpr auto value = Meta::front<spi_list>::value;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->ctrlb.template add<value>();                
                }
            }
        };

        template<typename CCList, AVR::Concepts::AtTiny2 MCU>
        struct StaticMapper<CCList, MCU> final {
        private:
            static constexpr auto mcu_pmux = getBaseAddr<typename MCU::Portmux>;
            using usart_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::usart::Mapper, CCList>>;
            using tca_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::tca::Mapper, CCList>>;
            using spi_list = Meta::filter<Meta::nonVoid, Meta::transform_type<detail::spi::Mapper, CCList>>;
            
            static_assert(Meta::size_v<usart_list> <= 4);
            static_assert(Meta::size_v<tca_list> <= 1);
            static_assert(Meta::size_v<spi_list> <= 1);
            
        public:
            static inline void init() {
                if constexpr(Meta::size_v<usart_list> > 0) {
                    constexpr auto value = Meta::value_or_v<usart_list>;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->ctrlb.template set<value>();                
                }
                if constexpr(Meta::size_v<tca_list> > 0) {
                    constexpr auto value = Meta::front<tca_list>::value;
//                    tca_list::_;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->ctrlc.template set<value>();                
                }
                if constexpr(Meta::size_v<spi_list> > 0) {
                    constexpr auto value = Meta::front<spi_list>::value;
//                    std::integral_constant<decltype(value), value>::_;
                    mcu_pmux()->ctrlb.template add<value>();                
                }
            }
        };
        
        struct DynamicMapper {
            
        };
        
    }
}
