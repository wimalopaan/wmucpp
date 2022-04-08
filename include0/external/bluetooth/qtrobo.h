#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

// Proportionalwerte Kanäle 1-19
// $p[01][0-9]:d[\n\r]
// $p[01][0-9]:dd[\n\r]
// $p[01][0-9]:ddd[\n\r]

// Schalter on/off [01] 1-19
// $s[01][0-9]:[01][\n\r]

// $t[01][0-9]:[01][\n\r]

// Output
// Logging
// l[0-9]:<text>\n

// Gauge
// g[0-9]:<0-255>\n

// Value
// v[0-9]:<0-255>\n

// Wind Zeiger1(innen) Zeiger2(aussen) LängeZeiger1(innen)
// w[0-9]:<0-255>:<0-255>:<0-255>\n

namespace External {
    namespace QtRobo {
        using namespace etl;
        using namespace std;
        
        constexpr auto startSymbol = std::byte{'$'};
        
        constexpr auto propSymbol = std::byte{'p'};
        constexpr auto switchSymbol = std::byte{'s'};
        constexpr auto toggleSymbol = std::byte{'t'};

        constexpr auto logSymbol = std::byte{'l'};
        constexpr auto gaugeSymbol = std::byte{'g'};
        constexpr auto valueSymbol = std::byte{'v'};
        constexpr auto windSymbol = std::byte{'w'};
        
        constexpr auto pingSymbol = std::byte{'>'};
        constexpr auto pongSymbol = std::byte{'<'};
        
        constexpr auto lineEnds  = std::make_array('\n'_B, '\r'_B, '\0'_B);
        constexpr auto outSeperator = std::byte{':'};
        constexpr auto inSeperators = std::make_array(' '_B, ':'_B);
        
        template<typename Device, auto N>
        struct Logging final {
            template<typename... PP>
            inline static constexpr void outl(const PP&... pp) {
                etl::outl<Device>(Char(startSymbol), Char(logSymbol), uint8_t{N}, Char(outSeperator), pp...);
            }
        };
        template<typename Device, auto N>
        struct Gauge final {
            inline static constexpr void put(const uint8_t v) {
                outl<Device>(Char(startSymbol), Char(gaugeSymbol), uint8_t{N}, Char(outSeperator), v);
            }
        };
        template<typename Device, auto N, typename T>
        struct Wind final {
            inline static constexpr void put(const T& z1, const T& z2, const T& l1) {
                outl<Device>(Char(startSymbol), Char(windSymbol), uint8_t{N}, Char(outSeperator), z1, Char(outSeperator), z2, Char(outSeperator), l1);
            }
        };
        template<typename Device>
        struct Pong final {
            inline static constexpr void put() {
                outl<Device>(Char{startSymbol}, Char{pongSymbol});
            }
        };
        template<typename Device, auto N = 0>
        struct Toggle final {
            inline static constexpr void put(const bool v) {
                if (v) {
                    outl<Device>(Char{startSymbol}, Char{toggleSymbol}, uint8_t{N}, Char{outSeperator}, 1);
                }
                else {
                    outl<Device>(Char{startSymbol}, Char{toggleSymbol}, uint8_t{N}, Char{outSeperator}, 0);
                }
            }
            inline static constexpr void put(const uint8_t i, const bool v) {
                if (v) {
                    outl<Device>(Char{startSymbol}, Char{toggleSymbol}, i, Char{outSeperator}, 1);
                }
                else {
                    outl<Device>(Char{startSymbol}, Char{toggleSymbol}, i, Char{outSeperator}, 0);
                }
            }
        };
        template<typename Device, auto N>
        struct ValuePlot final {
            inline static constexpr void put(const uint8_t v) {
                outl<Device>(Char{startSymbol}, Char{valueSymbol}, uint8_t{N}, Char{outSeperator}, v);
            }
        };
        
        constexpr inline bool isDigit(const std::byte b) {
            return ((static_cast<uint8_t>(b) >= '0') && (static_cast<uint8_t>(b) <= '9'));
        }

        constexpr inline uint8_t asDigit(const std::byte b) {
            return (static_cast<uint8_t>(b) - '0');
        }

        template<uint8_t N, uint8_t NChannels = 16, typename Buffer = void>
        class ProtocollAdapter final {
            inline static constexpr uint8_t NumberOfChannels = NChannels;
            
            typedef uint_ranged<uint8_t, 0, NumberOfChannels - 1> index_type;
            
            enum class State : uint8_t {Undefined, Start, Number, Seperator, Value, Ping, 
                                        Wind, WindV1, WindV2, WindV3};
        public:
            enum class Target: uint8_t {Undefined, Prop, Switch, Toggle, Ping, Wind};

            using buffer_t = Buffer;
            
            inline static bool process(const std::byte b) { 
                if constexpr(!std::is_same_v<Buffer, void>) {
                    if (Buffer::isActive()) {
                        Buffer::push(b);
                        return true;
                    }
                }
                ++mBytes;
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
                    else if (b == windSymbol) {
                        state = State::Wind;
                        target = Target::Wind;
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
                    else if (etl::contains(lineEnds, b) && (target == Target::Toggle)) {
                        if (index < std::size(toggleValues)) {
                            toggleValues[index] = !toggleValues[index];
                            changed = true;
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
                if (lastIndex && (lastTarget != Target::Undefined)) {
                    f(lastTarget, lastIndex.toInt());
                    lastTarget = Target::Undefined;
                    lastIndex.setNaN();
                }
            }
            inline static void whenPinged(const auto& f) {
                if (pingReceived) {
                    f();
                    pingReceived = false;                    
                }
            }
            inline static void ratePeriodic() {}
            
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
            inline static etl::uint_ranged_NaN<uint8_t, 0, NumberOfChannels - 1> lastIndex;
        public:
            inline static std::array<std::array<int16_t, 3>, NumberOfChannels> windValues;
            inline static std::array<int16_t, NumberOfChannels> propValues;
            inline static std::array<uint8_t, NumberOfChannels> switchValues;
            inline static std::array<bool, NumberOfChannels> toggleValues;
        };
    }
}

