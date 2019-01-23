#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>
//#include "container/stringbuffer.h"
//#include "util/algorithm.h"
//#include "util/disable.h"
//#include "util/types.h"

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
    namespace RoboRemo {
        using namespace etl;
        using namespace std;
        
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
        
        inline bool isdigit(std::byte b) {
            return ((static_cast<uint8_t>(b) >= '0') && (static_cast<uint8_t>(b) <= '9'));
        }
        template<uint8_t N, uint8_t NChannels = 16>
        class ProtocollAdapter final {
            inline static constexpr uint8_t NumberOfChannels = NChannels;
            typedef uint_ranged<uint8_t, 0, NumberOfChannels - 1> index_type;
            enum class State : uint8_t {Undefined, Start, Prop, PropNumber, Space, Value};
        public:
            inline static bool process(std::byte b) { 
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
                            index = number;
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
                default:
                    break;
                }
                return true;
            }    
        private:
            inline static uint8_t number;
            inline static index_type index;
            inline static State state = State::Undefined;
        public:
            inline static std::array<uint8_t, NumberOfChannels> propValues;
        };
    }
}
