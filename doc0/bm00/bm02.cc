#include <mcu/avr.h>
#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <algorithm>
#include <initializer_list>
#include <etl/stringbuffer.h>
#include <etl/algorithm.h>
#include <mcu/pgm/pgmarray.h>
#include <mcu/pgm/pgmstring.h>

namespace  {
    struct Data {
        inline constexpr Data() : fl{}, by{}, ar{} {}
        inline explicit Data(const AVR::Pgm::Ptr<Data>& p) {
            //            memcpy_P((std::byte*)this, p.raw(), sizeof(Data));
//            for(auto src{p.raw()}; auto&& dst : std::span<uint8_t, sizeof(Data)>{this}) {
//                dst = pgm_read_byte(src++);
//            }
            AVR::Pgm::ByteRange pgm{p};
            std::byte* dst = reinterpret_cast<std::byte*>(this);
            std::copy(std::begin(pgm), std::end(pgm), dst);
//            auto dst = (uint8_t*)this;
//            auto src = (const uint8_t*) p.raw();
//            for(uint8_t i{0}; i < sizeof(Data); ++i) {
//                *dst = pgm_read_byte(src);
//                ++src;
//                ++dst;
//            }
        }
        uint32_t fl; // float member renders this type to be non-structural, which is strange. So, test it with another 4-byte type
        uint8_t by;
        std::array<uint8_t, 100> ar;
    };
    
    struct Generator {
        constexpr auto operator()() {
            std::array<Data, 1> d{};
            auto& data = d[0];
            data.fl = 1234;
            data.by = 123;
            for(size_t i{0}; auto& e : data.ar) {
                e = std::numeric_limits<decltype(data.ar)::value_type>::max() * (1.0 + sin((2.0 * 3.14 * i++) / data.ar.size())) / 2.0;
            }
            return d;
        }        
    };
    
    using pgm_type = typename AVR::Pgm::Util::Converter<Generator>::pgm_type;
    
    Data ram_data; // uninitialized
}

template<typename T, typename... V>
void construct_inplace(T& d, const V& ... v) {
    new (&d) T{v...};
}

int main() {
    construct_inplace(ram_data, pgm_type{}.ptr(0));
    
    return ram_data.ar[10];
}

