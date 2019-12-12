#pragma once

#include <cstdint>

#include "../../config.h"

#include <external/units/physical.h>

#include "groups.h"

namespace AVR {
    using namespace External::Units;
    
    namespace Util {
        
        namespace Timer {
            
            template<typename T>
            struct TimerSetupData final {
                const uint16_t prescaler = 0;
                const T ocr = 0;
                const hertz f{0};
                const bool isExact = false;
                explicit constexpr operator bool() const {
                    return (prescaler > 0) && (ocr > 0);
                }
            };
            
            // returns possible prescaler values
            template<typename CSBits, auto N>
            constexpr auto prescalerValues(const std::array<PrescalerPair<CSBits>, N>& a) {
                std::array<typename PrescalerPair<CSBits>::scale_type, N> values{};
                for(uint8_t i = 0; i < N; ++i) {
                    values[i] = a[i].scale;
                }
                return values;
            }
            
            // return clock select bits for specific Prescale
            template<uint16_t Prescale, typename CSBits, int N>
            constexpr auto bitsFrom(const std::array<PrescalerPair<CSBits>, N>& a) {
                using bits_type = CSBits;
                for(const auto& pair: a) {
                    if (pair.scale == Prescale) {
                        return pair.bits;
                    }
                }
                return bits_type{0};
            }
        }
        
    }
}
