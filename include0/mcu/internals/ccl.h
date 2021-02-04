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
            template<uint8_t N> struct Ac;
            template<uint8_t N> struct Zcd;
            template<uint8_t N> struct Usart;
            struct Spi0;
            template<uint8_t N> struct Tca0;
            template<uint8_t N> struct Tcb;
            template<uint8_t N> struct Tcd;
            
            namespace detail {
                template<typename In, uint8_t Position, typename MCU = DefaultMcuType> struct map_input;

                template<AVR::Concepts::AtDa32 MCU> struct map_input<Mask, 0, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Feedback, 0, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Link, 0, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Event<A>, 0, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Event<B>, 0, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<IOPin, 0, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Ac<0>, 0, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Zcd<0>, 0, MCU> : std::integral_constant<std::byte, 0x07_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Usart<0>, 0, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Spi0, 0, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tca0<0>, 0, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tca0<1>, 0, MCU> : std::integral_constant<std::byte, 0x0b_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tcb<0>, 0, MCU> : std::integral_constant<std::byte, 0x0c_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tcd<0>, 0, MCU> : std::integral_constant<std::byte, 0x0d_B> {};

                template<AVR::Concepts::AtDa32 MCU> struct map_input<Mask, 1, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Feedback, 1, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Link, 1, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Event<A>, 1, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Event<B>, 1, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<IOPin, 1, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Ac<1>, 1, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Zcd<1>, 1, MCU> : std::integral_constant<std::byte, 0x07_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Usart<1>, 1, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Spi0, 1, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tca0<0>, 1, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tca0<1>, 1, MCU> : std::integral_constant<std::byte, 0x0b_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tcb<1>, 1, MCU> : std::integral_constant<std::byte, 0x0c_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tcd<0>, 1, MCU> : std::integral_constant<std::byte, 0x0d_B> {};

                template<AVR::Concepts::AtDa32 MCU> struct map_input<Mask, 2, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Feedback, 2, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Link, 2, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Event<A>, 2, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Event<B>, 2, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<IOPin, 2, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Ac<2>, 2, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Zcd<2>, 2, MCU> : std::integral_constant<std::byte, 0x07_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Usart<2>, 2, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Spi0, 2, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tca0<0>, 2, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tca0<1>, 2, MCU> : std::integral_constant<std::byte, 0x0b_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tcb<2>, 2, MCU> : std::integral_constant<std::byte, 0x0c_B> {};
                template<AVR::Concepts::AtDa32 MCU> struct map_input<Tcd<0>, 2, MCU> : std::integral_constant<std::byte, 0x0d_B> {};
                
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Mask, 0, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Feedback, 0, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Link, 0, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Event<A>, 0, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Event<B>, 0, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<IOPin, 0, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Ac0, 0, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Usart<0>, 0, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Spi0, 0, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Tca0<0>, 0, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Tcb<0>, 0, MCU> : std::integral_constant<std::byte, 0x0c_B> {};

                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Mask, 0, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Feedback, 0, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Link, 0, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Event<A>, 0, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Event<B>, 0, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<IOPin, 0, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Ac0, 0, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tcb<0>, 0, MCU> : std::integral_constant<std::byte, 0x07_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tca0<0>, 0, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tcd<0>, 0, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Usart<0>, 0, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Spi0, 0, MCU> : std::integral_constant<std::byte, 0x0b_B> {};
                
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Mask, 1, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Feedback, 1, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Link, 1, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Event<A>, 1, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Event<B>, 1, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<IOPin, 1, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Ac0, 1, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Usart<1>, 1, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Spi0, 1, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Tca0<1>, 1, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Tcb<1>, 1, MCU> : std::integral_constant<std::byte, 0x0c_B> {};

                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Mask, 1, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Feedback, 1, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Link, 1, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Event<A>, 1, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Event<B>, 1, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<IOPin, 1, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Ac0, 1, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tcb<0>, 1, MCU> : std::integral_constant<std::byte, 0x07_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tca0<1>, 1, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tcd<0>, 1, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Usart<0>, 1, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Spi0, 1, MCU> : std::integral_constant<std::byte, 0x0b_B> {};
                
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Mask, 2, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Feedback, 2, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Link, 2, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Event<A>, 2, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Event<B>, 2, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<IOPin, 2, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Ac0, 2, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Usart<2>, 2, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Spi0, 2, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Tca0<2>, 2, MCU> : std::integral_constant<std::byte, 0x0a_B> {};
                template<AVR::Concepts::AtMega0 MCU> struct map_input<Tcb<2>, 2, MCU> : std::integral_constant<std::byte, 0x0c_B> {};
                
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Mask, 2, MCU> : std::integral_constant<std::byte, 0x00_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Feedback, 2, MCU> : std::integral_constant<std::byte, 0x01_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Link, 2, MCU> : std::integral_constant<std::byte, 0x02_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Event<A>, 2, MCU> : std::integral_constant<std::byte, 0x03_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Event<B>, 2, MCU> : std::integral_constant<std::byte, 0x04_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<IOPin, 2, MCU> : std::integral_constant<std::byte, 0x05_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Ac0, 2, MCU> : std::integral_constant<std::byte, 0x06_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tcb<0>, 2, MCU> : std::integral_constant<std::byte, 0x07_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tca0<2>, 2, MCU> : std::integral_constant<std::byte, 0x08_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Tcd<0>, 2, MCU> : std::integral_constant<std::byte, 0x09_B> {};
                template<AVR::Concepts::AtTiny1 MCU> struct map_input<Spi0, 2, MCU> : std::integral_constant<std::byte, 0x0b_B> {};
            }
        }
        namespace Output {
            namespace detail {
                template<uint8_t N, typename MCU = DefaultMcuType> struct map_output;
                
                template<AVR::Concepts::AtMega0 MCU>
                struct map_output<0, MCU> {
                    using pin_type = AVR::Pin<AVR::Port<AVR::A>, 3>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct map_output<1, MCU> {
                    using pin_type = AVR::Pin<AVR::Port<AVR::C>, 3>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct map_output<2, MCU> {
                    using pin_type = AVR::Pin<AVR::Port<AVR::D>, 3>;
                };
                template<AVR::Concepts::AtMega0 MCU>
                struct map_output<3, MCU> {
                    using pin_type = AVR::Pin<AVR::Port<AVR::F>, 3>;
                };
                
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
            static constexpr uint8_t number = N;
            static constexpr auto mcu_ccl = getBaseAddr<typename MCU::Ccl>;
            
            inline static void init(std::byte truthTable) {
                constexpr auto insel0 = Input::detail::map_input<In0, 0, MCU>::value;                
                constexpr auto insel1 = (Input::detail::map_input<In1, 1, MCU>::value << 4);                
                constexpr auto insel01 = insel0 | insel1;
                constexpr auto insel2 = Input::detail::map_input<In2, 2, MCU>::value;            
              
                detail::LutTransaction<N> lut0t1([&]{
                    mcu_ccl()->luts.raw[N].ctrlb = insel01;
                    mcu_ccl()->luts.raw[N].ctrlc = insel2;
                    mcu_ccl()->luts.raw[N].truth = truthTable;
                });
            }  

            inline static void enable() {
                mcu_ccl()->luts.raw[N].ctrla.template add<MCU::Ccl::Lut0CtrlA_t::enable | MCU::Ccl::Lut0CtrlA_t::outenable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            }
            inline static void disable() {
                mcu_ccl()->luts.raw[N].ctrla.template clear<MCU::Ccl::Lut0CtrlA_t::enable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
                mcu_ccl()->luts.raw[N].ctrla.template clear<MCU::Ccl::Lut0CtrlA_t::outenable, etl::DisbaleInterrupt<etl::NoDisableEnable>>();
            }
        };
        
        template<typename Lut>
        struct LutOutPin {
            using lut = Lut;
            using pin_type = Output::detail::map_output<Lut::number>::pin_type;
            
            template<typename Dir>
            inline static void dir() {
                static_assert(std::is_same_v<Dir, AVR::Output>);
                pin_type::template dir<Dir>();                
            }
            
            inline static void on() {
                pin_type::on();
                Lut::disable();
            }

            inline static void off() {
                pin_type::off();
                Lut::disable();
            }

            inline static void lutOn() {
                Lut::enable();
            }
        };
    }   
}
