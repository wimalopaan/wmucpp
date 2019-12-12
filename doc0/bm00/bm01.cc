#include <mcu/avr.h>
#include <array>
#include <cmath>
#include <memory>
#include <etl/stringbuffer.h>
#include <etl/algorithm.h>
#include <mcu/pgm/pgmarray.h>
#include <mcu/pgm/pgmstring.h>

namespace  {
    struct SineGenerator {
        constexpr auto operator()() {
            std::array<uint8_t, 100> data{};
            for(size_t i{0}; auto& e : data) {
                e = std::numeric_limits<decltype(data)::value_type>::max() * (1.0 + sin((2.0 * 3.14 * i++) / data.size())) / 2.0;
            }
            return data;
        }        
    };
    struct Data {
        Data(const Data&) = delete;
        Data& operator=(const Data&) = delete;
        
        inline Data() {}
                
        using pgm_type = typename AVR::Pgm::Util::Converter<SineGenerator>::pgm_type;
        
//        using pgm_type = AVR::Pgm::Array<uint8_t, 
//            1,   2,   3,   4,   5,   6,   7,   8,   9,  10,
//           11,  12,  13,  14,  15,  16,  17,  18,  19,  20,
//           21,  22,  23,  24,  25,  26,  27,  28,  29,  30,
//           31,  32,  33,  34,  35,  36,  37,  38,  39,  40,
//           41,  42,  43,  44,  45,  46,  47,  48,  49,  50,
//           51,  52,  53,  54,  55,  56,  57,  58,  59,  60,
//           61,  62,  63,  64,  65,  66,  67,  68,  69,  70,
//           71,  72,  73,  74,  75,  76,  77,  78,  79,  80,
//           81,  82,  83,  84,  85,  86,  87,  88,  89,  90,
//           91,  92,  93,  94,  95,  96,  97,  98,  99, 100
//        >;
        
        inline void reset() {
            new(this) Data();
        }
        
        float fl{1.23f};
        uint8_t by{123};
//        etl::StringBuffer<100u> s{"hello"_pgm};
        std::array<pgm_type::value_type, pgm_type::size()> ar = std::to_array(pgm_type{});
    };
    
    Data ram_data;
}

int main() {
    ram_data.reset();
    
    return ram_data.ar[10];
}
