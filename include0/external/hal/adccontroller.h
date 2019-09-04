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

#include "mcu/internals/adc.h"

namespace External {
    namespace Hal {

        template<typename MCUAdc, typename ListOfChannels, typename MCU = DefaultMcuType>
        class AdcController;
        
        template<typename MCUAdc, auto... Channels, AVR::Concepts::AtMega MCU>
        class AdcController<MCUAdc, Meta::NList<Channels...>, MCU> final {
            enum class State : uint8_t {Start, Converting, ConversionComplete};
            
        public:
            typedef MCUAdc mcu_adc_type;
            typedef typename MCUAdc::value_type value_type;
            inline static constexpr uint8_t channels[] = {Channels...};
            inline static constexpr uint8_t NumberOfChannels = sizeof... (Channels);    
            inline static constexpr auto VRef = MCUAdc::VRef;
            
            typedef etl::uint_ranged<uint8_t, 0, NumberOfChannels - 1> index_type;     
            
            static_assert(NumberOfChannels <= 8);
            static_assert(NumberOfChannels >  0);
            
            inline static void init() {
                MCUAdc::init();
                MCUAdc::channel(channels[0]);
            }
            
            inline static void periodic() {
                static State state = State::Start;
                switch(state) {
                case State::Start:
                    MCUAdc::startConversion();
                    state = State::Converting;
                    break;
                case State::Converting:
                    if (MCUAdc::conversionReady()) {
                        state = State::ConversionComplete;
                    }
                    break;
                case State::ConversionComplete:
                    values[mActualChannel] = MCUAdc::value();
                    MCUAdc::channel(channels[++mActualChannel]);
                    state = State::Start;
                    break;
                }
            }
            
            // ranged
//            inline static typename MCUAdc::value_type value(uint8_t index) {
//                assert(index < NumberOfChannels);
//                return values[index];
//            }
            inline static typename MCUAdc::value_type value(index_type index) {
                return values[index.toInt()];
            }
            
            // ranged
//            inline static typename MCUAdc::voltage_type voltage(uint8_t index) {
//                return MCUAdc::toVoltage(value(index));
//            }
            
        private:
            inline static typename MCUAdc::value_type values[NumberOfChannels] = {};
            inline static etl::uint_ranged_circular<uint8_t, 0, NumberOfChannels - 1> mActualChannel;
        };

    
        template<typename MCUAdc, auto... Channels, AVR::Concepts::At01Series MCU>
        class AdcController<MCUAdc, Meta::NList<Channels...>, MCU> final {
            enum class State : uint8_t {Start, Converting, ConversionComplete};

            template<typename T, typename MMCU>
            using ChannelPinMapper = AVR::ADC::ChannelPinMapper<T, MMCU>;
            
            template<typename T>
            using map_channel_to_pin = typename ChannelPinMapper<T, MCU>::pin_type;
            
            using ch_list = typename Meta::NList<Channels...>::list_type;
            using pin_list = Meta::transform<map_channel_to_pin, ch_list>;
            
//            pin_list::_;
            
        public:
            using mcu_adc_type = MCUAdc ;
            using value_type = typename MCUAdc::value_type;
            
            inline static constexpr uint8_t channels[] = {Channels...};
            inline static constexpr uint8_t NumberOfChannels = sizeof... (Channels);    
            inline static constexpr auto VRef = MCUAdc::VRef;
            
            using index_type = etl::uint_ranged_circular<uint8_t, 0, NumberOfChannels - 1>;     
            
            static_assert(NumberOfChannels <= 14, "too much channels");
            static_assert(NumberOfChannels >  0, "use at least one channel");
            static_assert(Meta::is_set_v<typename Meta::NList<Channels...>::list_type>, "the channels must be different");
            
            inline static void init() {
                []<typename... Pin>(Meta::List<Pin...>) {
                    (Pin::template attributes<Meta::List<AVR::Attributes::DigitalDisable<>>>(), ...);
                    (Pin::template pullup<false>(), ...);
                }(pin_list{});
                MCUAdc::init();
                MCUAdc::channel(channels[0]);
            }
            
            inline static void periodic() {
                static State state = State::Start;
                switch(state) {
                case State::Start:
                    MCUAdc::startConversion();
                    state = State::Converting;
                    break;
                case State::Converting:
                    MCUAdc::whenConversionReady([&]{
                        state = State::ConversionComplete;
                    });
                    break;
                case State::ConversionComplete:
                    values[mActualChannel] = MCUAdc::value();
                    MCUAdc::channel(channels[++mActualChannel]);
                    state = State::Start;
                    break;
                }
            }
            
            inline static typename MCUAdc::value_type value(index_type index) {
                return values[index.toInt()];
            }
            
        private:
            inline static value_type values[NumberOfChannels] = {};
            inline static index_type mActualChannel;
        };
    
    }
}
