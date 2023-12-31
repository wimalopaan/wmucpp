#include <mcu/avr.h>
#include <array>
#include <cmath>
#include <memory>
#include <functional>
#include <etl/stringbuffer.h>
#include <etl/algorithm.h>
#include <mcu/pgm/pgmarray.h>
#include <mcu/pgm/pgmstring.h>

#if 1
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
//    struct Data {
//        Data(const Data&) = delete;
//        Data& operator=(const Data&) = delete;
        
//        inline Data() {}
                
//        using pgm_type = typename AVR::Pgm::Util::Converter<SineGenerator>::pgm_type;
        
////        using pgm_type = AVR::Pgm::Array<uint8_t, 
////            1,   2,   3,   4,   5,   6,   7,   8,   9,  10,
////           11,  12,  13,  14,  15,  16,  17,  18,  19,  20,
////           21,  22,  23,  24,  25,  26,  27,  28,  29,  30,
////           31,  32,  33,  34,  35,  36,  37,  38,  39,  40,
////           41,  42,  43,  44,  45,  46,  47,  48,  49,  50,
////           51,  52,  53,  54,  55,  56,  57,  58,  59,  60,
////           61,  62,  63,  64,  65,  66,  67,  68,  69,  70,
////           71,  72,  73,  74,  75,  76,  77,  78,  79,  80,
////           81,  82,  83,  84,  85,  86,  87,  88,  89,  90,
////           91,  92,  93,  94,  95,  96,  97,  98,  99, 100
////        >;
        
//        inline void reset() {
//            new(this) Data();
//        }
        
//        float fl{1.23f};
//        uint8_t by{123};
////        etl::StringBuffer<100u> s{"hello"_pgm};
//        std::array<pgm_type::value_type, pgm_type::size()> ar = std::to_array(pgm_type{});
//    };
    
//    Data ram_data;


    using pgm_type = typename AVR::Pgm::Util::Converter<SineGenerator>::pgm_type;

}

#endif

//uint8_t digit_sum(uint32_t n) {
//  uint8_t sum = 0;
//  do {
//    uint8_t x = n % 10u;
//    n /= 10u;
//    sum += x;
//  } while(n != 0);
//  return sum;
//}
 
//#define VPORTA_OUTTGL VPORTA_IN

//struct R {
//private:
//    struct P {
//        friend class R;
//        void operator=(const int val) && {
//            VPORTA_IN = val;    
//        }
//    private:
//        explicit P(R& r) : r{r} {}
//        R& r;
//    };
//public:
//    P operator*() {
//        return P{*this};
//    }
//};

//template<typename T> 
//uint8_t digit_sum2(T n) {
////    decltype(n)::_;
//  uint8_t sum = 0;
//  do {
//    uint8_t x = n % 10;
//    n /= 10;
//    sum += x;
//  } while(n != T{0});

//  return sum; 
//}

int main() {
    while(true) {
        for(const auto& v : pgm_type{}) {
              
        }
    }
    
    
//    VPORTA_INTFLAGS = 0x01;
//    R r;
//    *r = 1;

//    auto& rr = *r;
//    rr = 1;
    
//    return digit_sum2(42UL);
    
//    ram_data.reset();
    
//    return ram_data.ar[10];
}
