#pragma once

#include <cstdint>
#include <cstddef>

namespace GPS {

    using std::byte;
    
#include "../../../3rdparty/tinygps/TinyGPS.h"
#include "../../../3rdparty/tinygps/TinyGPS.cc"

    
    template<uint16_t N>
    struct GpsProtocollAdapter {
        inline static bool process(std::byte) { // from isr only
            return true;
        }    
        
        inline static TinyGPSPlus gps;
        
    };
}
