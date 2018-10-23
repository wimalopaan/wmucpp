#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>
#include "container/stringbuffer.h"
#include "util/algorithm.h"
#include "util/disable.h"
#include "util/types.h"


namespace FrSky{
    template<uint8_t N>
    struct ProtocollAdapter final {
        enum class State : uint8_t {Undefined, Head, Data};
        inline static constexpr auto head = std::byte{0x7e};
        inline static constexpr auto prim = std::byte{0xfe};
        inline static constexpr auto end  = std::byte{0x7e};
        inline static bool process(std::byte b) { // from isr only
            switch (state) {
            case State::Undefined:
                if (b == head) {
                    state = State::Head;
                }
                break;
            case State::Head:
                if (b == prim) {
                    state = State::Data;
                    index = 0;
                }
                break;
            case State::Data:
                if (index < data.size) {
                    data[index++] = b;
                }
                else {
                    if (b == end) {
                        state = State::Undefined;
                    }
                }
                break;
            default:
                assert(false);
                break;
            }
            return true;
        }    
//    private:
        inline static std::array<std::byte, 8> data;        
        inline static State state = State::Undefined;
        inline static uint8_t index = 0;
    };
}
