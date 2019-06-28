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
           template<AVR::Concepts::Letter L, uint8_t N> struct Pin;
           template<uint8_t N, typename K> struct Usart;
           template<typename K> struct Spi0;
           template<typename K> struct Tca0;
           template<uint8_t N> struct Tcb;
        }
        
        namespace detail {
            template<typename Gen, typename CHNumber> struct map_generator;

            template<uint8_t N> struct map_generator<Generators::Updi, std::integral_constant<uint8_t, N>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::updi;
            };
            template<uint8_t N> struct map_generator<Generators::Rtc<Generators::Kind::Ovf>, std::integral_constant<uint8_t, N>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_ovf;
            };
            template<uint8_t N> struct map_generator<Generators::Rtc<Generators::Kind::Cmp>, std::integral_constant<uint8_t, N>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_cmp;
            };
            template<> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 0>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 1>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 2>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 3>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 4>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 5>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<> struct map_generator<Generators::PitDiv<8192>, std::integral_constant<uint8_t, 6>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };
            template<> struct map_generator<Generators::PitDiv<512>, std::integral_constant<uint8_t, 7>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit0;
            };

            template<> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 0>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 1>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 2>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 3>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 4>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 5>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<> struct map_generator<Generators::PitDiv<4096>, std::integral_constant<uint8_t, 6>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };
            template<> struct map_generator<Generators::PitDiv<256>, std::integral_constant<uint8_t, 7>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit1;
            };

            template<> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 0>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 1>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 2>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 3>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 4>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 5>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<> struct map_generator<Generators::PitDiv<2048>, std::integral_constant<uint8_t, 6>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };
            template<> struct map_generator<Generators::PitDiv<128>, std::integral_constant<uint8_t, 7>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit2;
            };

            template<> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 0>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 1>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 2>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 3>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 4>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 5>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<> struct map_generator<Generators::PitDiv<1024>, std::integral_constant<uint8_t, 6>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };
            template<> struct map_generator<Generators::PitDiv<64>, std::integral_constant<uint8_t, 7>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::rtc_pit3;
            };

            template<uint8_t L, uint8_t N> requires(L < 4) struct map_generator<Generators::Lut<L>, std::integral_constant<uint8_t, N>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t{uint8_t(AVR::Series0::Events::Generator_t::ccl_lut0) + N};
            };
        
            template<uint8_t N> struct map_generator<Generators::Ac0<Generators::Kind::Out>, std::integral_constant<uint8_t, N>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::ac0_out;
            };
            template<uint8_t N> struct map_generator<Generators::Adc0<Generators::Kind::Ready>, std::integral_constant<uint8_t, N>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::adc0_rdy;
            };
            
            template<AVR::Concepts::Letter L, uint8_t PNum, uint8_t N> requires(PNum < 8) struct map_generator<Generators::Pin<L, PNum>, std::integral_constant<uint8_t, N>> {
                inline static constexpr auto value = AVR::Series0::Events::Generator_t::adc0_rdy;
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
                template<typename U>
                struct user_to_index;
                
                template<> struct user_to_index<Lut<0, A>> : std::integral_constant<uint8_t, 0>{};
                template<> struct user_to_index<Lut<0, B>> : std::integral_constant<uint8_t, 1>{};
                template<> struct user_to_index<Lut<1, A>> : std::integral_constant<uint8_t, 2>{};
                template<> struct user_to_index<Lut<1, B>> : std::integral_constant<uint8_t, 3>{};
                template<> struct user_to_index<Lut<2, A>> : std::integral_constant<uint8_t, 4>{};
                template<> struct user_to_index<Lut<2, B>> : std::integral_constant<uint8_t, 5>{};
                template<> struct user_to_index<Lut<3, A>> : std::integral_constant<uint8_t, 6>{};
                template<> struct user_to_index<Lut<3, B>> : std::integral_constant<uint8_t, 7>{};
                template<> struct user_to_index<Adc0> : std::integral_constant<uint8_t, 8>{};
                template<> struct user_to_index<EvOut<A>> : std::integral_constant<uint8_t, 9>{};
                template<> struct user_to_index<EvOut<B>> : std::integral_constant<uint8_t, 10>{};
                template<> struct user_to_index<EvOut<C>> : std::integral_constant<uint8_t, 11>{};
                template<> struct user_to_index<EvOut<D>> : std::integral_constant<uint8_t, 12>{};
                template<> struct user_to_index<EvOut<E>> : std::integral_constant<uint8_t, 13>{};
                template<> struct user_to_index<EvOut<F>> : std::integral_constant<uint8_t, 14>{};
                template<> struct user_to_index<Usart<0>> : std::integral_constant<uint8_t, 15>{};
                template<> struct user_to_index<Usart<1>> : std::integral_constant<uint8_t, 16>{};
                template<> struct user_to_index<Usart<2>> : std::integral_constant<uint8_t, 17>{};
                template<> struct user_to_index<Usart<3>> : std::integral_constant<uint8_t, 18>{};
                template<> struct user_to_index<Tca0> : std::integral_constant<uint8_t, 19>{};
                template<> struct user_to_index<Tcb<0>> : std::integral_constant<uint8_t, 20>{};
                template<> struct user_to_index<Tcb<1>> : std::integral_constant<uint8_t, 21>{};
                template<> struct user_to_index<Tcb<2>> : std::integral_constant<uint8_t, 22>{};
                template<> struct user_to_index<Tcb<3>> : std::integral_constant<uint8_t, 23>{};
                            
            }
        }   
        
        template<uint8_t Number, typename Generator, AVR::Concepts::At01Series MCU = DefaultMcuType>
        requires(Number < 8)
        struct Channel {
            template<typename Channels, typename Routes> friend struct Router;
            
            using number_type = std::integral_constant<uint8_t, Number>;
            using generator_type = Generator;

            private:
            inline static constexpr auto gen_code = detail::map_generator<generator_type, number_type>::value;

            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;
            
            inline static void setup() {
                mcu_evsys()->channels[Number].template set<gen_code>();
            }    
        };
        
        template<typename C, typename User, AVR::Concepts::At01Series MCU = DefaultMcuType>
        requires requires(C){typename C::generator_type;}
        struct Route {
            template<typename Channels, typename Routes> friend struct Router;
            using channel_number_type = typename C::number_type;
            static_assert(channel_number_type::value < 8, "wrong channel");
            
            inline static constexpr uint8_t userNumber = Users::detail::user_to_index<User>::value;
            
            
        private:
            static constexpr auto mcu_evsys = getBaseAddr<typename MCU::Events>;
            static_assert(userNumber < std::tuple_size<decltype(MCU::Events::users)>::value, "wrong user index");

            inline static void setup() {
                mcu_evsys()->users[userNumber].template set<typename MCU::Events::Channel_t{channel_number_type::value}>();
            }
        };
        
        template<typename... CC> struct Channels : Meta::List<CC...>{};
        template<typename... RR> struct Routes : Meta::List<RR...>{};
        
        template<typename Channels, typename Routes>
        struct Router {
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
        };
        
    }
}
