/* WMuCpp - Bare Metal C++ 
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

#include "../common/concepts.h"

namespace AVR {
    namespace Ccl {
        namespace Input {
            struct Mask;
            struct Feedback;
            struct Link;
            template<AVR::Concepts::Letter> struct Event;
            struct IOPin;
            struct Ac0;
            template<uint8_t N> struct Usart;
            struct Spi0;
            template<uint8_t N> struct Tca0;
            template<uint8_t N> struct Tcb;
            
            namespace detail {
                template<typename In, uint8_t Position> struct map_input;
                
                template<> struct map_input<Mask, 0> : std::integral_constant<std::byte, 0x00_B> {};
                template<> struct map_input<Feedback, 0> : std::integral_constant<std::byte, 0x01_B> {};
                template<> struct map_input<Link, 0> : std::integral_constant<std::byte, 0x02_B> {};
                template<> struct map_input<Event<A>, 0> : std::integral_constant<std::byte, 0x03_B> {};
                template<> struct map_input<Event<B>, 0> : std::integral_constant<std::byte, 0x04_B> {};
                template<> struct map_input<IOPin, 0> : std::integral_constant<std::byte, 0x05_B> {};
                template<> struct map_input<Ac0, 0> : std::integral_constant<std::byte, 0x06_B> {};
                template<> struct map_input<Usart<0>, 0> : std::integral_constant<std::byte, 0x08_B> {};
                template<> struct map_input<Spi0, 0> : std::integral_constant<std::byte, 0x09_B> {};
                template<> struct map_input<Tca0<0>, 0> : std::integral_constant<std::byte, 0x0a_B> {};
                template<> struct map_input<Tcb<0>, 0> : std::integral_constant<std::byte, 0x0c_B> {};
                                
                template<> struct map_input<Mask, 1> : std::integral_constant<std::byte, 0x00_B> {};
                template<> struct map_input<Feedback, 1> : std::integral_constant<std::byte, 0x01_B> {};
                template<> struct map_input<Link, 1> : std::integral_constant<std::byte, 0x02_B> {};
                template<> struct map_input<Event<A>, 1> : std::integral_constant<std::byte, 0x03_B> {};
                template<> struct map_input<Event<B>, 1> : std::integral_constant<std::byte, 0x04_B> {};
                template<> struct map_input<IOPin, 1> : std::integral_constant<std::byte, 0x05_B> {};
                template<> struct map_input<Ac0, 1> : std::integral_constant<std::byte, 0x06_B> {};
                template<> struct map_input<Usart<1>, 1> : std::integral_constant<std::byte, 0x08_B> {};
                template<> struct map_input<Spi0, 1> : std::integral_constant<std::byte, 0x09_B> {};
                template<> struct map_input<Tca0<1>, 1> : std::integral_constant<std::byte, 0x0a_B> {};
                template<> struct map_input<Tcb<1>, 1> : std::integral_constant<std::byte, 0x0c_B> {};

                template<> struct map_input<Mask, 2> : std::integral_constant<std::byte, 0x00_B> {};
                template<> struct map_input<Feedback, 2> : std::integral_constant<std::byte, 0x01_B> {};
                template<> struct map_input<Link, 2> : std::integral_constant<std::byte, 0x02_B> {};
                template<> struct map_input<Event<A>, 2> : std::integral_constant<std::byte, 0x03_B> {};
                template<> struct map_input<Event<B>, 2> : std::integral_constant<std::byte, 0x04_B> {};
                template<> struct map_input<IOPin, 2> : std::integral_constant<std::byte, 0x05_B> {};
                template<> struct map_input<Ac0, 2> : std::integral_constant<std::byte, 0x06_B> {};
                template<> struct map_input<Usart<2>, 2> : std::integral_constant<std::byte, 0x08_B> {};
                template<> struct map_input<Spi0, 2> : std::integral_constant<std::byte, 0x09_B> {};
                template<> struct map_input<Tca0<2>, 2> : std::integral_constant<std::byte, 0x0a_B> {};
                template<> struct map_input<Tcb<2>, 2> : std::integral_constant<std::byte, 0x0c_B> {};
            }
        }
        
        namespace detail {
            template<typename MCU = DefaultMcuType>
            struct CclTransaction {
                static constexpr auto mcu_ccl = getBaseAddr<typename MCU::Ccl>;
                CclTransaction() {
                    mcu_ccl()->ctrla.template clear<MCU::Ccl::CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                }
                ~CclTransaction() {
                    mcu_ccl()->ctrla.template add<MCU::Ccl::CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                    
                }
            };
            template<uint8_t N, typename MCU = DefaultMcuType>
            struct LutTransaction {
                static constexpr auto mcu_ccl = getBaseAddr<typename MCU::Ccl>;
                LutTransaction(auto f) {
                    detail::CclTransaction cclt1;
                    mcu_ccl()->luts.raw[N].ctrla.template clear<MCU::Ccl::Lut0CtrlA_t::enable | MCU::Ccl::Lut0CtrlA_t::outenable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                    f();
                    mcu_ccl()->luts.raw[N].ctrla.template add<MCU::Ccl::Lut0CtrlA_t::filter_synch | MCU::Ccl::Lut0CtrlA_t::enable | MCU::Ccl::Lut0CtrlA_t::outenable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                }
            };
        }
        
        
        // component number type
        template<uint8_t N, typename In0, typename In1, typename In2, typename MCU = DefaultMcuType> 
        struct SimpleLut {
            static constexpr auto mcu_ccl = getBaseAddr<typename MCU::Ccl>;
            
            inline static void init(std::byte truthTable) {
                constexpr auto insel0 = Input::detail::map_input<In0, 0>::value;                
                constexpr auto insel1 = (Input::detail::map_input<In1, 1>::value << 4);                
                constexpr auto insel01 = insel0 | insel1;
                constexpr auto insel2 = Input::detail::map_input<In2, 2>::value;            
              
                detail::LutTransaction<N> lut0t1([&]{
                    mcu_ccl()->luts.raw[N].ctrlb = insel01;
                    mcu_ccl()->luts.raw[N].ctrlc = insel2;
                    mcu_ccl()->luts.raw[N].truth = truthTable;
                });
                
            }              
        };
        
        template<typename Lut>
        struct LutOutPin {
            
            inline static void on() {
                
            }

            inline static void off() {
                
            }

            inline static void lutOn() {
                
            }
            
        };
        
        
        
    }   
}
