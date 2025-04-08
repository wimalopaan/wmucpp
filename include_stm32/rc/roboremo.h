/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <cstddef>
#include <array>
#include <algorithm>

#include "etl/ranged.h"

// Proportionalwerte Kanäle 1-19
// $p[01][0-9] d[\n\r]
// $p[01][0-9] dd[\n\r]
// $p[01][0-9] ddd[\n\r]

// Schalter on/off [01] 1-19
// $s[01][0-9] [01][\n\r]

// Output
// Logging
// l[0-9] <text>
// Gauge
// g[0-9] <0-255>

namespace External {
    namespace QtRobo {
        using namespace etl;
        using namespace std;
        using namespace etl::literals;
        
        constexpr auto startSymbol = std::byte{'$'};
        
        constexpr auto propSymbol = std::byte{'p'};
        constexpr auto switchSymbol = std::byte{'s'};
        constexpr auto toggleSymbol = std::byte{'t'};
        constexpr auto menuSymbol = std::byte{'m'};
        constexpr auto connectSymbol = std::byte{'c'}; // kind of passthru

        constexpr auto logSymbol = std::byte{'l'};
        constexpr auto gaugeSymbol = std::byte{'g'};
        constexpr auto valueSymbol = std::byte{'v'};
        constexpr auto windSymbol = std::byte{'w'};
        
        constexpr auto pingSymbol = std::byte{'>'};
        constexpr auto pongSymbol = std::byte{'<'};
        
        constexpr auto lineEnds  = std::array{'\n'_B, '\r'_B, '\0'_B};
        constexpr auto outSeperator = std::byte{':'};
        constexpr auto inSeperators = std::array{' '_B, ':'_B};

        constexpr auto passthruStart1 = std::byte{0x00};
        constexpr auto passthruStart2 = std::byte{0xff};


        // template<typename Device, auto N>
        // struct Logging final {
        //     template<typename... PP>
        //     inline static constexpr void outl(const PP&... pp) {
        //         etl::outl<Device>(Char(startSymbol), Char(logSymbol), uint8_t{N}, Char(outSeperator), pp...);
        //     }
        // };
        // template<typename Device, auto N>
        // struct Gauge final {
        //     inline static constexpr void put(const uint8_t v) {
        //         outl<Device>(Char(startSymbol), Char(gaugeSymbol), uint8_t{N}, Char(outSeperator), v);
        //     }
        // };
        // template<typename Device, auto N, typename T>
        // struct Wind final {
        //     inline static constexpr void put(const T& z1, const T& z2, const T& l1) {
        //         outl<Device>(Char(startSymbol), Char(windSymbol), uint8_t{N}, Char(outSeperator), z1, Char(outSeperator), z2, Char(outSeperator), l1);
        //     }
        // };
        // template<typename Device>
        // struct Pong final {
        //     inline static constexpr void put() {
        //         outl<Device>(Char(startSymbol), Char(pongSymbol));
        //     }
        // };
        // template<typename Device, auto N = 0>
        // struct Toggle final {
        //     inline static constexpr void put(const bool v) {
        //         if (v) {
        //             outl<Device>(Char(startSymbol), Char(toggleSymbol), uint8_t{N}, Char(outSeperator), 1);
        //         }
        //         else {
        //             outl<Device>(Char(startSymbol), Char(toggleSymbol), uint8_t{N}, Char(outSeperator), 0);
        //         }
        //     }
        //     inline static constexpr void put(const uint8_t i, const bool v) {
        //         if (v) {
        //             outl<Device>(Char(startSymbol), Char(toggleSymbol), i, Char(outSeperator), 1);
        //         }
        //         else {
        //             outl<Device>(Char(startSymbol), Char(toggleSymbol), i, Char(outSeperator), 0);
        //         }
        //     }
        // };
        // template<typename Device, auto N>
        // struct ValuePlot final {
        //     inline static constexpr void put(const uint8_t v) {
        //         outl<Device>(Char(startSymbol), Char(valueSymbol), uint8_t{N}, Char(outSeperator), v);
        //     }
        // };
        
        constexpr inline bool isDigit(const std::byte b) {
            return ((static_cast<uint8_t>(b) >= '0') && (static_cast<uint8_t>(b) <= '9'));
        }

        constexpr inline uint8_t asDigit(const std::byte b) {
            return (static_cast<uint8_t>(b) - '0');
        }

        template<uint8_t N, typename Timer, typename PassThru, typename SendBack = void, uint8_t NChannels = 16, typename Buffer = void, typename Dbg = void>
        struct ProtocollAdapter final {
            inline static constexpr uint8_t NumberOfChannels = NChannels;
            using index_type = etl::ranged<0, NumberOfChannels - 1>;
            enum class State : uint8_t {Undefined, Start, Number, Seperator, Value, Ping, 
                                        Wind, WindV1, WindV2, WindV3,
                                        PassThru1, PassThru2,
                                        CPassThru1, CPassThru2, CPassThruEnd1, CPassThruEnd2,
                                        };
            enum class Target: uint8_t {Undefined, Prop, Switch, Toggle, Ping, Wind, Menu};

            static inline constexpr External::Tick<Timer> byteTimeout{1000ms};

            using buffer_t = Buffer;
            
            inline static bool process(const std::byte b) { 
                if constexpr(!std::is_same_v<Buffer, void>) {
                    if (Buffer::isActive()) {
                        Buffer::push(b);
                        return true;
                    }
                }
                mByteTimer.reset();
                ++mBytes;
                switch (state) {
                case State::Undefined:
                    if (b == startSymbol) {
                        state = State::Start;
                    }
                    else if (b == passthruStart1) {
                        PassThru::start();
                        state = State::PassThru1;
                    }
                    break;
                case State::PassThru1:
                    if (b == passthruStart2) {
                        PassThru::put(passthruStart1);
                        PassThru::put(passthruStart2);
                        state = State::PassThru2;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::PassThru2:
                    PassThru::put(b);
                    break;
                case State::CPassThru1:
                    if (etl::contains(lineEnds, b)) {
                        PassThru::start();
                        SendBack::put('$'_B);
                        SendBack::put('l'_B);
                        SendBack::put('0'_B);
                        SendBack::put(' '_B);
                        SendBack::put('1'_B);
                        SendBack::put('\n'_B);
                        state = State::CPassThru2;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::CPassThru2:
                    if (b == startSymbol) {
                        state = State::CPassThruEnd1;
                    }
                    else {
                        PassThru::put(b);
                    }
                    break;
                case State::CPassThruEnd1:
                    if (b == connectSymbol) {
                        state = State::CPassThruEnd2;
                    }
                    else {
                        PassThru::put(startSymbol);
                        PassThru::put(b);
                        state = State::CPassThru2;
                    }
                    break;
                case State::CPassThruEnd2:
                    if (etl::contains(lineEnds, b)) {
                        PassThru::stop();
                        SendBack::put('$'_B);
                        SendBack::put('l'_B);
                        SendBack::put('0'_B);
                        SendBack::put(' '_B);
                        SendBack::put('0'_B);
                        SendBack::put('\n'_B);
                        state = State::Undefined;
                    }
                    else {
                        state = State::CPassThru2;
                    }
                    break;
                case State::Start:
                    number = 0;
                    if (b == propSymbol) {
                        state = State::Number;
                        target = Target::Prop;
                    }
                    else if (b == switchSymbol) {
                        state = State::Number;
                        target = Target::Switch;
                    }
                    else if (b == toggleSymbol) {
                        state = State::Number;
                        target = Target::Toggle;
                    }
                    else if (b == pingSymbol) {
                        state = State::Ping;
                        target = Target::Undefined;
                    }
                    else if (b == windSymbol) {
                        state = State::Wind;
                        target = Target::Wind;
                    }
                    else if (b == menuSymbol) {
                        state = State::Number;
                        target = Target::Menu;
                    }
                    else if (b == connectSymbol) {
                        state = State::CPassThru1;
                        target = Target::Undefined;
                    }
                    else {
                        state = State::Undefined;
                        target = Target::Undefined;
                    }
                    break;
                case State::Wind:
                    if (isDigit(b)) {
                        number *= 10;
                        number += asDigit(b);
                    }
                    else if (etl::contains(inSeperators, b)) {
                        state = State::WindV1;
                        index.set(number);
                        number = 0;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::Number:
                    if (isDigit(b)) {
                        number *= 10;
                        number += asDigit(b);
                    }
                    else if (etl::contains(inSeperators, b)) {
                        state = State::Value;
                        index.set(number);
                        number = 0;
                        sign = 1;
                    }
                    else if (etl::contains(lineEnds, b)) {
                        if (target == Target::Toggle) {
                            index.set(number);
                            if (index < std::size(toggleValues)) {
                                toggleValues[index] = !toggleValues[index];
                                changed = true;
                                lastIndex = index;
                                lastTarget = Target::Toggle;
                                state = State::Undefined;
                            }
                        }
                        else if (target == Target::Menu) {
                            changed = true;
                            state = State::Undefined;
                            lastIndex.setNaN();
                            lastTarget = Target::Menu;                            
                        }
                        else {
                            state = State::Undefined;                            
                        }
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::WindV1:
                    if (isDigit(b)) {
                        number *= 10;
                        number += asDigit(b);
                    }
                    else if (b == std::byte{'-'}) {
                        sign = -1;
                    }
                    else if (b == std::byte{'+'}) {
                    }
                    else if (etl::contains(inSeperators, b)) {
                        w0 = number * sign;
                        number = 0;
                        sign = 1;
                        state = State::WindV2;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::WindV2:
                    if (isDigit(b)) {
                        number *= 10;
                        number += asDigit(b);
                    }
                    else if (b == std::byte{'-'}) {
                        sign = -1;
                    }
                    else if (b == std::byte{'+'}) {
                    }
                    else if (etl::contains(inSeperators, b)) {
                        w1 = number * sign;
                        number = 0;
                        sign = 1;
                        state = State::WindV3;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::WindV3:
                    if (isDigit(b)) {
                        number *= 10;
                        number += asDigit(b);
                    }
                    else if (b == std::byte{'-'}) {
                        sign = -1;
                    }
                    else if (b == std::byte{'+'}) {
                    }
                    else if (etl::contains(lineEnds, b)) {
                        if (index < std::size(windValues)) {
                            windValues[index][0] = w0;
                            windValues[index][1] = w1;
                            windValues[index][2] = number * sign;
                            number = 0;
                            sign = 1;
                            changed = true;
                            state = State::Undefined;
                        }
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::Value:
                    if (isDigit(b)) {
                        number *= 10;
                        number += asDigit(b);
                    }
                    else if (b == std::byte{'-'}) {
                        sign = -1;
                    }
                    else if (b == std::byte{'+'}) {
                    }
                    else if (etl::contains(lineEnds, b)) {
                        if (target == Target::Prop) {
                            if (index < std::size(propValues)) {
                                propValues[index] = number * sign;
                                changed = true;
                                lastTarget = Target::Prop;
                                lastIndex = index;
                                ++mPackages;
                            }
                        }
                        else if (target == Target::Switch) {
                            if (index < std::size(switchValues)) {
                                switchValues[index] = number;
                                changed = true;
                                lastTarget = Target::Switch;
                                lastIndex = index;
                                ++mPackages;
                            }
                        }
                        else if (target == Target::Toggle) {
                            if (index < std::size(toggleValues)) {
                                toggleValues[index] = !toggleValues[index];
                                changed = true;
                                lastTarget = Target::Toggle;
                                lastIndex = index;
                                ++mPackages;
                            }
                        }
                        else if (target == Target::Menu) {
                            changed = true;
                            lastIndex = index;
                            lastTarget = Target::Menu;
                            ++mPackages;
                        }
                        state = State::Undefined;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::Ping:
                    if (etl::contains(  lineEnds, b)) {
                        state = State::Undefined;
                        pingReceived = true;
                    }
                    break;
                case State::Seperator:
                    break;
                default:
                    break;
                }
                return true;
            }  
            inline static void whenChanged(const auto& f) {
                if (changed) {
                    f();
                    changed = false;
                }
            }
            inline static void whenTargetChanged(const auto& f) {
                if (changed && (lastTarget != Target::Undefined)) {
                    f(lastTarget, lastIndex);
                    lastTarget = Target::Undefined;
                    lastIndex.setNaN();
                    changed = false;
                }
            }
            inline static void whenPinged(const auto& f) {
                if (pingReceived) {
                    f();
                    pingReceived = false;                    
                }
            }
            inline static void ratePeriodic() {
                ++mByteTimer;
                mByteTimer.on(byteTimeout, []{
                    if (state == State::PassThru2) {
                        PassThru::stop();
                        state = State::Undefined;
                    }
                });                  
            }
            
            inline static uint16_t packages() {
                return mPackages;
            }
            inline static uint16_t bytes() {
                return mBytes;
            }
            inline static void resetStats() {
                mBytes = mPackages = 0;
            }
        private:
            static inline External::Tick<Timer> mByteTimer{};
            inline static uint16_t mBytes{};
            inline static uint16_t mPackages{};
            
            inline static int16_t w0;
            inline static int16_t w1;
            
            inline static int8_t sign;
            inline static int16_t number;
            inline static index_type index;
            inline static State state = State::Undefined;
            inline static Target target = Target::Undefined;
            inline static bool changed = false;
            inline static bool pingReceived = false;
            inline static Target lastTarget = Target::Undefined;
            inline static etl::ranged_NaN<0, NumberOfChannels - 1> lastIndex;
        public:
            inline static std::array<std::array<int16_t, 3>, NumberOfChannels> windValues;
            inline static std::array<int16_t, NumberOfChannels> propValues;
            inline static std::array<uint8_t, NumberOfChannels> switchValues;
            inline static std::array<bool, NumberOfChannels> toggleValues;
        };
    }
    namespace RoboRemo {
        using namespace etl::literals;
        
#if 0
        template<typename Device, auto N>
        class Logging {
            template<typename... PP>
            inline static constexpr void outl(const PP&... pp) {
                outl<Device>(Char{'l'}, uint8_t{N}, Char{' '}, pp...);
            }
        };
        template<typename Device, auto N>
        class Gauge {
            inline static constexpr void put(uint8_t v) {
                outl<Device>(Char{'g'}, uint8_t{N}, Char{' '}, v);
            }
        };
        template<typename Device, auto N>
        class ValuePlot {
            inline static constexpr void put(uint8_t v) {
                outl<Device>(Char{'v'}, uint8_t{N}, Char{' '}, v);
            }
        };
#endif   
        inline bool isdigit(std::byte b) {
            return ((static_cast<uint8_t>(b) >= '0') && (static_cast<uint8_t>(b) <= '9'));
        }

        template<uint8_t N, uint8_t NChannels = 16, typename Buffer = void>
        class ProtocollAdapter final {
            inline static constexpr uint8_t NumberOfChannels = NChannels;
            typedef etl::ranged<0, NumberOfChannels - 1> index_type;
            enum State {Undefined, Start, Prop, PropNumber, Space, Value};
        public:
            using buffer_t = Buffer;

            inline static bool process(const std::byte b) { 
                switch (state) {
                case State::Undefined:
                    if (b == '$'_B) {
                        state = State::Start;
                    }
                    break;
                case State::Start:
                    if (b == 'p'_B) {
                        state = State::Prop;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::Prop:
                    if (isdigit(b)) {
                        number = 10 * (static_cast<uint8_t>(b) - '0');
                        state = State::PropNumber;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::PropNumber:
                    if (isdigit(b)) {
                        number += (static_cast<uint8_t>(b) - '0');
                        if (number < NumberOfChannels) {
                            index = index_type{number};
                            state = State::Space;
                        }
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::Space:
                    if (b == ' '_B) {
                        state = State::Value;
                        number = 0;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::Value:
                    if (isdigit(b)) {
                        number *= 10;
                        number += (static_cast<uint8_t>(b) - '0');
                    }
                    else if ((b == '\n'_B) || (b == '\r'_B)) {
                        if (number <= std::numeric_limits<uint8_t>::max()) {
                            propValues[index] = number;
                        }
                        state = State::Undefined;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
#ifdef NDEBUG
                default:
                    __builtin_unreachable();
                    break;
#endif     
                }
                return true;
            }
            template<typename C>
            inline static void copyChangedChannels(C& out) {
                for(uint8_t i = 0; (i < NumberOfChannels) && (i < out.size()); ++i) {
                    if (propValues[i] != propValuesPrevious[i]) {
                        propValuesPrevious[i] = propValues[i];
                        out[i] = propValues[i];
                    }
                }
            }
            template<typename C>
            inline static void copyChangedAltData(C& out) {
                for(uint8_t i = 0; (i < altValues.size()) && (i < out.size()); ++i) {
                    if (altValues[i] != altValuesPrevious[i]) {
                        altValuesPrevious[i] = altValues[i];
                        out[i] = altValues[i];
                    }
                }
            }
            
            
            inline static void ratePeriodic() {}
        private:
            inline static uint8_t number;
            inline static index_type index;
            inline static State state = State::Undefined;
        public:
            using values_t = std::array<uint16_t, NumberOfChannels>;
            inline static values_t propValues;
            inline static values_t propValuesPrevious;

            using altdata_t = std::array<uint8_t, 256>;
            inline static altdata_t altValues;
            inline static altdata_t altValuesPrevious;
        };
    }
}
