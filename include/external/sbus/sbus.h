#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

#include "util/types.h"

namespace SBUS {
   
    
    class Container {
    public:
        using value_type = uint_ranged<uint16_t, 0, 2047>; // 11-Bit
        using index_type = uint_ranged<uint8_t, 0, 15>; // 16 Kanäle
        
        const value_type& operator[](index_type index) const {
            return values[index];
        }
        value_type& operator[](index_type index) {
            return values[index];
        }
        void updateValues() {
            
        }   
        void updateRaw() {
            
        }
    private:
        std::array<value_type, 16> values;
        std::array<std::byte, 25> raw;
    };
    
    template<uint8_t N>
    class ProtocollAdapter {
    public:
        inline static bool process(std::byte b) { // from isr only
            return true;
        }        
    private:
        inline static SBUS::Container data;
    };
}
