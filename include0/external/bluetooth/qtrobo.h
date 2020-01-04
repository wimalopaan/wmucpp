#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

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
        
        constexpr auto seperator = std::byte{':'};
        constexpr auto startSymbol = std::byte{'$'};
        
        constexpr auto propSymbol = std::byte{'p'};
        constexpr auto switchSymbol = std::byte{'s'};
        constexpr auto toggleSymbol = std::byte{'t'};

        constexpr auto logSymbol = std::byte{'l'};
        constexpr auto gaugeSymbol = std::byte{'g'};
        constexpr auto valueSymbol = std::byte{'v'};
        
        constexpr auto pingSymbol = std::byte{'>'};
        constexpr auto pongSymbol = std::byte{'<'};
        
        constexpr auto lineEnds = std::make_array('\n'_B, '\r'_B, '\0'_B);
        
        template<typename Device, auto N>
        struct Logging final {
            template<typename... PP>
            inline static constexpr void outl(const PP&... pp) {
                etl::outl<Device>(Char{startSymbol}, Char{logSymbol}, uint8_t{N}, Char{seperator}, pp...);
            }
        };
        template<typename Device, auto N>
        struct Gauge final {
            inline static constexpr void put(uint8_t v) {
                outl<Device>(Char{startSymbol}, Char{gaugeSymbol}, uint8_t{N}, Char{seperator}, v);
            }
        };
        template<typename Device>
        struct Pong final {
            inline static constexpr void put() {
                outl<Device>(Char{startSymbol}, Char{pongSymbol});
            }
        };
        template<typename Device, auto N>
        struct Toggle final {
            inline static constexpr void put(bool v) {
                if (v) {
                    outl<Device>(Char{startSymbol}, Char{toggleSymbol}, uint8_t{N}, Char{seperator}, 1);
                }
                else {
                    outl<Device>(Char{startSymbol}, Char{toggleSymbol}, uint8_t{N}, Char{seperator}, 0);
                }
            }
        };
        template<typename Device, auto N>
        struct ValuePlot final {
            inline static constexpr void put(uint8_t v) {
                outl<Device>(Char{startSymbol}, Char{valueSymbol}, uint8_t{N}, Char{seperator}, v);
            }
        };
        
        constexpr inline bool isDigit(std::byte b) {
            return ((static_cast<uint8_t>(b) >= '0') && (static_cast<uint8_t>(b) <= '9'));
        }

        constexpr inline uint8_t asDigit(std::byte b) {
            return (static_cast<uint8_t>(b) - '0');
        }

        template<uint8_t N, uint8_t NChannels = 16>
        class ProtocollAdapter final {
            inline static constexpr uint8_t NumberOfChannels = NChannels;
            typedef uint_ranged<uint8_t, 0, NumberOfChannels - 1> index_type;
            enum class State : uint8_t {Undefined, Start, Number, Seperator, Value, Ping};
            enum class Target: uint8_t {Undefined, Prop, Switch, Toggle, Ping};
        public:
            inline static bool process(std::byte b) { 
                switch (state) {
                case State::Undefined:
                    if (b == startSymbol) {
                        state = State::Start;
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
                    else {
                        state = State::Undefined;
                        target = Target::Undefined;
                    }
                    break;
                case State::Number:
                    if (isDigit(b)) {
                        number *= 10;
                        number += asDigit(b);
                    }
                    else if (b == seperator) {
                        state = State::Value;
                        index.set(number);
                        number = 0;
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
                    else if (etl::contains(lineEnds, b)) {
                        if (number <= std::numeric_limits<uint8_t>::max()) {
                            if (target == Target::Prop) {
                                if (index < std::size(propValues)) {
                                    propValues[index] = number;
                                    changed = true;
                                }
                            }
                            else if (target == Target::Switch) {
                                if (index < std::size(switchValues)) {
                                    switchValues[index] = number;
                                    changed = true;
                                }
                            }
                            else if (target == Target::Toggle) {
                                if (index < std::size(switchValues)) {
                                    toggleValues[index] = !toggleValues[index];
                                    changed = true;
                                }
                            }
                        }
                        state = State::Undefined;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                case State::Ping:
                    if (etl::contains(lineEnds, b)) {
                        state = State::Undefined;
                        pingReceived = true;
                    }
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
            inline static void whenPinged(const auto& f) {
                if (pingReceived) {
                    f();
                    pingReceived = false;                    
                }
            }
        private:
            inline static uint8_t number;
            inline static index_type index;
            inline static State state = State::Undefined;
            inline static Target target = Target::Undefined;
            inline static bool changed = false;
            inline static bool pingReceived = false;
        public:
            inline static std::array<uint8_t, NumberOfChannels> propValues;
            inline static std::array<bool, NumberOfChannels> switchValues;
            inline static std::array<bool, NumberOfChannels> toggleValues;
        };
    }
}

