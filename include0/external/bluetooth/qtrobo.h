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

        constexpr std::array<std::byte, 3> lineEnds = {'\n'_B, '\r'_B, '\0'_B};
        
        template<typename Device, auto N>
        struct Logging {
            template<typename... PP>
            inline static constexpr void outl(const PP&... pp) {
                etl::outl<Device>(Char{'l'}, uint8_t{N}, seperator, pp...);
            }
        };
        template<typename Device, auto N>
        struct Gauge {
            inline static constexpr void put(uint8_t v) {
                outl<Device>(Char{'g'}, uint8_t{N}, seperator, v);
            }
        };
        template<typename Device, auto N>
        struct ValuePlot {
            inline static constexpr void put(uint8_t v) {
                outl<Device>(Char{'v'}, uint8_t{N}, seperator, v);
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
            enum class State : uint8_t {Undefined, Start, Number, Seperator, Value};
            enum class Target: uint8_t {Undefined, Prop, Switch};
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
                        index = number;
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
                                }
                            }
                            else if (target == Target::Switch) {
                                if (index < std::size(switchValues)) {
                                    switchValues[index] = number;
                                }
                            }
                        }
                        state = State::Undefined;
                    }
                    else {
                        state = State::Undefined;
                    }
                    break;
                default:
                    break;
                }
                return true;
            }    
        private:
            inline static uint8_t number;
            inline static index_type index;
            inline static State state = State::Undefined;
            inline static Target target = Target::Undefined;
        public:
            inline static std::array<uint8_t, NumberOfChannels> propValues;
            inline static std::array<bool, NumberOfChannels> switchValues;
        };
    }
}

