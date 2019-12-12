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

#include "../common/concepts.h"

namespace AVR {
    namespace Event {
        namespace Generators {
            namespace Kind {
                struct Ovf;
                struct Cmp;
                struct Out;
                struct Ready;
                struct Xck;
                struct Sck;
                struct Hunf;
                struct Cmp0;
                struct Cmp1;
                struct Cmp2;
            } 
           struct Updi;
           template<typename K> struct Rtc;
           template<uint16_t Div> struct PitDiv;
           template<uint8_t N> struct Lut;
           template<typename K> struct Ac0;
           template<typename K> struct Adc0;
//           template<AVR::Concepts::Letter L, uint8_t N> struct Pin;
           template<AVR::Concepts::Pin P> struct Pin;
           template<uint8_t N, typename K> struct Usart;
           template<typename K> struct Spi0;
           template<typename K> struct Tca0;
           template<uint8_t N> struct Tcb;
        }
        
        namespace detail {
            template<typename Gen, typename CHNumber, typename MCU = DefaultMcuType> struct map_generator;

            template<uint8_t N, AVR::Concepts::AtMega0 MCU> struct map_generator<void, std::integral_constant<uint8_t, N>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{0};
            };

            template<uint8_t N, AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::Updi, std::integral_constant<uint8_t, N>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::updi;
            };
            template<uint8_t N, AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::Rtc<Generators::Kind::Ovf>, std::integral_constant<uint8_t, N>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_ovf;
            };
            template<uint8_t N, AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::Rtc<Generators::Kind::Cmp>, std::integral_constant<uint8_t, N>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_cmp;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 0>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 1>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 2>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 3>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 4>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 5>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 6>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 7>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };

            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 0>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 1>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 2>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 3>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 4>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 5>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 6>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 7>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };

            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 0>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 1>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 2>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 3>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 4>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 5>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 6>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 7>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };

            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 0>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 1>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 2>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 3>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 4>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 5>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 6>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 7>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };

            template<uint8_t L, uint8_t N, AVR::Concepts::AtMega0 MCU> requires(L < 4) struct map_generator<Generators::Lut<L>, std::integral_constant<uint8_t, N>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::ccl_lut0) + N};
            };
        
            template<uint8_t N, AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::Ac0<Generators::Kind::Out>, std::integral_constant<uint8_t, N>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::ac0_out;
            };
            template<uint8_t N, AVR::Concepts::AtMega0 MCU> struct map_generator<Generators::Adc0<Generators::Kind::Ready>, std::integral_constant<uint8_t, N>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::adc0_rdy;
            };
            
            template<uint8_t N, uint8_t C, AVR::Concepts::AtMega0 MCU>  requires((C == 0) || (C == 1)) struct map_generator<Generators::Pin<AVR::Pin<AVR::Port<AVR::A>, N>>, std::integral_constant<uint8_t, C>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::port0_pin0) + N};
            };
            template<uint8_t N, uint8_t C, AVR::Concepts::AtMega0 MCU>  requires((C == 2) || (C == 3)) struct map_generator<Generators::Pin<AVR::Pin<AVR::Port<AVR::C>, N>>, std::integral_constant<uint8_t, C>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::port0_pin0) + N};
            };
            template<uint8_t N, uint8_t C, AVR::Concepts::AtMega0 MCU>  requires((C == 4) || (C == 5)) struct map_generator<Generators::Pin<AVR::Pin<AVR::Port<AVR::E>, N>>, std::integral_constant<uint8_t, C>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::port0_pin0) + N};
            };
            template<uint8_t N, uint8_t C, AVR::Concepts::AtMega0 MCU>  requires((C == 0) || (C == 1)) struct map_generator<Generators::Pin<AVR::Pin<AVR::Port<AVR::B>, N>>, std::integral_constant<uint8_t, C>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::port1_pin0) + N};
            };
            template<uint8_t N, uint8_t C, AVR::Concepts::AtMega0 MCU>  requires((C == 2) || (C == 3)) struct map_generator<Generators::Pin<AVR::Pin<AVR::Port<AVR::D>, N>>, std::integral_constant<uint8_t, C>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::port1_pin0) + N};
            };
            template<uint8_t N, uint8_t C, AVR::Concepts::AtMega0 MCU>  requires((C == 4) || (C == 5)) struct map_generator<Generators::Pin<AVR::Pin<AVR::Port<AVR::F>, N>>, std::integral_constant<uint8_t, C>, MCU> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::port1_pin0) + N};
            };

            template<uint8_t N, uint8_t C, AVR::Concepts::AtTiny1 MCU>  requires((C == 0)) struct map_generator<Generators::Pin<AVR::Pin<AVR::Port<AVR::A>, N>>, std::integral_constant<uint8_t, C>, MCU> {
                inline static constexpr auto value = AVR::Series1::Events::AsyncGenerator_t{uint8_t(AVR::Series1::Events::AsyncGenerator_t::port_pin0) + N};
            };
        }
        
        namespace Users {
            template<uint8_t N, AVR::Concepts::Letter L> struct Lut;
            struct Adc0;
            template<AVR::Concepts::Letter L> struct EvOut;
            template<uint8_t N> struct Usart;
            struct Tca0;
            template<uint8_t N> struct Tcb;
            
            namespace detail {
                template<typename U, typename MCU = DefaultMcuType> struct user_to_index;
                
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<0, A>, MCU> : std::integral_constant<uint8_t, 0>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<0, B>, MCU> : std::integral_constant<uint8_t, 1>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<1, A>, MCU> : std::integral_constant<uint8_t, 2>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<1, B>, MCU> : std::integral_constant<uint8_t, 3>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<2, A>, MCU> : std::integral_constant<uint8_t, 4>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<2, B>, MCU> : std::integral_constant<uint8_t, 5>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<3, A>, MCU> : std::integral_constant<uint8_t, 6>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Lut<3, B>, MCU> : std::integral_constant<uint8_t, 7>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Adc0, MCU> : std::integral_constant<uint8_t, 8>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<EvOut<A>, MCU> : std::integral_constant<uint8_t, 9>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<EvOut<B>, MCU> : std::integral_constant<uint8_t, 10>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<EvOut<C>, MCU> : std::integral_constant<uint8_t, 11>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<EvOut<D>, MCU> : std::integral_constant<uint8_t, 12>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<EvOut<E>, MCU> : std::integral_constant<uint8_t, 13>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<EvOut<F>, MCU> : std::integral_constant<uint8_t, 14>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Usart<0>, MCU> : std::integral_constant<uint8_t, 15>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Usart<1>, MCU> : std::integral_constant<uint8_t, 16>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Usart<2>, MCU> : std::integral_constant<uint8_t, 17>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Usart<3>, MCU> : std::integral_constant<uint8_t, 18>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Tca0, MCU> : std::integral_constant<uint8_t, 19>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Tcb<0>, MCU> : std::integral_constant<uint8_t, 20>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Tcb<1>, MCU> : std::integral_constant<uint8_t, 21>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Tcb<2>, MCU> : std::integral_constant<uint8_t, 22>{};
                template<AVR::Concepts::AtMega0 MCU> struct user_to_index<Tcb<3>, MCU> : std::integral_constant<uint8_t, 23>{};

                template<AVR::Concepts::AtTiny1 MCU> struct user_to_index<Tcb<0>, MCU> : std::integral_constant<uint8_t, 0>{};
            
            
            }
        }   


        template<typename Channels, typename Routes, typename MCU = DefaultMcuType> struct Router;

        template<uint8_t Number, typename Generator, typename MCU = DefaultMcuType> struct Channel;
        
        template<uint8_t Number, typename Generator, AVR::Concepts::AtMega0 MCU>
        requires(Number < 8)
        struct Channel<Number, Generator, MCU> {
            template<typename Channels, typename Routes, typename > friend struct Router;
            
            using number_type = std::integral_constant<uint8_t, Number>;
            using generator_type = Generator;

            private:
            inline static constexpr auto gen_code = detail::map_generator<generator_type, number_type>::value;

            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;
            
            inline static void setup() {
                mcu_evsys()->channels[Number].template set<gen_code>();
            }    
        };
        template<uint8_t Number, typename Generator, AVR::Concepts::AtTiny1 MCU>
        requires(Number < 6)
        struct Channel<Number, Generator, MCU> {
            template<typename Channels, typename Routes, typename > friend struct Router;
            
            using number_type = std::integral_constant<uint8_t, Number>;
            using generator_type = Generator;

            private:
            inline static constexpr auto gen_code = detail::map_generator<generator_type, number_type>::value;

//            std::integral_constant<uint8_t, uint8_t(gen_code)>::_;
            
            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;
            
            inline static void setup() {
                if constexpr(Number < 4) {
                    mcu_evsys()->async_channels[Number].template set<gen_code>();
                }
                else if constexpr((Number == 4) || (Number == 5)) {
                    mcu_evsys()->sync_channels[Number - 4].template set<gen_code>();
                }
                else {
                    static_assert(std::false_v<MCU>);
                }
            }    
        };

        template<typename C, typename User, typename MCU = DefaultMcuType> struct Route;
        
        template<typename C, typename User, AVR::Concepts::AtMega0 MCU>
        requires requires(C){typename C::generator_type;}
        struct Route<C, User, MCU> {
            template<typename Channels, typename Routes, typename> friend struct Router;
            using channel_number_type = typename C::number_type;
            static_assert(channel_number_type::value < 8, "wrong channel");
            
            inline static constexpr uint8_t userNumber = Users::detail::user_to_index<User>::value;
            
        private:
            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;
            static_assert(userNumber < std::tuple_size<decltype(MCU::Events::users)>::value, "wrong user index");

            inline static void setup() {
                mcu_evsys()->users[userNumber].template set<typename MCU::Events::Channel_t{channel_number_type::value + 1}>();
            }
        };
        template<typename C, typename User, AVR::Concepts::AtTiny1 MCU>
        requires requires(C){typename C::generator_type;}
        struct Route<C, User, MCU> {
            template<typename Channels, typename Routes, typename> friend struct Router;
            using channel_number_type = typename C::number_type;
            
            inline static constexpr uint8_t channel_number = channel_number_type::value;
            static_assert(channel_number < 6, "wrong channel");
            
            inline static constexpr uint8_t userNumber = Users::detail::user_to_index<User>::value;
            
//            std::integral_constant<uint8_t, userNumber>::_;
            
        private:
            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;
            static_assert(userNumber < std::tuple_size<decltype(MCU::Events::async_users)>::value, "wrong user index");

            inline static void setup() {
                if constexpr(channel_number < 4) {
//                    std::integral_constant<uint8_t, channel_number + 3>::_;
                    mcu_evsys()->async_users[userNumber].template set<typename MCU::Events::AsyncChannel_t{channel_number + 3}>();
                }
            }
        };
        
        template<typename... CC> struct Channels : Meta::List<CC...>{};
        template<typename... RR> struct Routes : Meta::List<RR...>{};
        
        template<typename Channels, typename Routes, AVR::Concepts::AtMega0 MCU>
        struct Router<Channels, Routes, MCU> {
            static_assert(Meta::size_v<Channels> < 8, "too much channels");
            static_assert(Meta::size_v<Routes> < 24, "too much users");
            static_assert(Meta::is_set_v<Routes>, "duplicate route");
            static_assert(Meta::is_set_v<Channels>, "duplicate channel");
            
            template<typename C>
            struct get_channel_t {
                using type = typename C::number_type;  
            };
            using channel_number_list = Meta::transform_type<get_channel_t, Channels>;
//            channel_number_list::_;
            static_assert(Meta::is_set_v<channel_number_list>, "duplicate channels in generation definition");
            
            inline static void init() {
                []<typename... C>(Meta::List<C...>){
                    (C::setup(), ...);
                }(Channels{});
                []<typename... R>(Meta::List<R...>){
                    (R::setup(), ...);
                }(Routes{});
            }
            
            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;

            template<uint8_t N>
            inline static void strobe() {
                static_assert(N < 8, "wrong channel, must be 0 to 7");
                constexpr auto code = typename MCU::Events::Strobe_t{1 << N};
                mcu_evsys()->strobe.template set<code>();
            }
        };

        template<typename Channels, typename Routes, AVR::Concepts::AtTiny1 MCU>
        struct Router<Channels, Routes, MCU> {
            static_assert(Meta::size_v<Channels> < 6, "too much channels");
            static_assert(Meta::size_v<Routes> < 13, "too much users");
            static_assert(Meta::is_set_v<Routes>, "duplicate route");
            static_assert(Meta::is_set_v<Channels>, "duplicate channel");
            
            template<typename C>
            struct get_channel_t {
                using type = typename C::number_type;  
            };
            using channel_number_list = Meta::transform_type<get_channel_t, Channels>;
//            channel_number_list::_;
            static_assert(Meta::is_set_v<channel_number_list>, "duplicate channels in generation definition");
            
            inline static void init() {
                []<typename... C>(Meta::List<C...>){
                    (C::setup(), ...);
                }(Channels{});
                []<typename... R>(Meta::List<R...>){
                    (R::setup(), ...);
                }(Routes{});
            }
            
            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;

            template<uint8_t N>
            inline static void strobe() {
                static_assert(N < 8, "wrong channel, must be 0 to 7");
                constexpr auto code = typename MCU::Events::Strobe_t{1 << N};
                mcu_evsys()->strobe.template set<code>();
            }
        };
    }
}
