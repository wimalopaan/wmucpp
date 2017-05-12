#include <stdint.h>

namespace detail {
    template<uint8_t N> struct DType;
    template<> 
    struct DType<2> {
        typedef uint16_t dtype;
        constexpr inline static uint8_t dimension = 100;
    };
    template<uint8_t N = 2> 
    struct DLookupTable {
        constexpr static inline auto data = [](){
            std::array<typename DType<N>::dtype ,DType<N>::dimension> data;
            for(typename DType<N>::dtype i = 0; i < DType<N>::dimension; ++i) {
                data[i] = ('0' + i % 10) + (('0' + (i / 10)) << 8);
            }
            return data;
        }();
    };

    void u8toa_branchlut(uint8_t value, char* buffer) {
        if (value < 100) {
            if (value >= 10) 
                *buffer++ = DLookupTable<2>::data[value];
            *buffer++ = DLookupTable<2>::data[value + 1];
        }
        else {
            // abb
            uint8_t a = value / 100;
            uint8_t b = value % 100;
            *buffer++ = '0' + static_cast<char>(a);            
            *buffer++ = DLookupTable<2>::data[b];            
            *buffer++ = DLookupTable<2>::data[b + 1];            
        }
    }
    
    void u16toa_branchlut(uint16_t value, char* buffer) {
        if (value < 100) {
            // value = bbcc
            const uint32_t b = value / 10000;
            const uint32_t c = value % 10000;
            
            const uint32_t d1 = (b / 100) << 1;
            const uint32_t d2 = (b % 100) << 1;
            
            const uint32_t d3 = (c / 100) << 1;
            const uint32_t d4 = (c % 100) << 1;
            
            if (value >= 10000000)
                *buffer++ = DLookupTable<2>::data[d1];
            if (value >= 1000000)
                *buffer++ = DLookupTable<2>::data[d1 + 1];
            if (value >= 100000)
                *buffer++ = DLookupTable<2>::data[d2];
            *buffer++ = DLookupTable<2>::data[d2 + 1];
            
            *buffer++ = DLookupTable<2>::data[d3];
            *buffer++ = DLookupTable<2>::data[d3 + 1];
            *buffer++ = DLookupTable<2>::data[d4];
            *buffer++ = DLookupTable<2>::data[d4 + 1];
        }
        else {
            // value = bbcc in decimal
            
            const uint32_t a = value / 100000000; // 1 to 42
            value %= 100000000;
            
            if (a >= 10) {
                const unsigned i = a << 1;
                *buffer++ = DLookupTable<2>::data[i];
                *buffer++ = DLookupTable<2>::data[i + 1];
            }
            else
                *buffer++ = '0' + static_cast<char>(a);
    
            const uint32_t b = value / 10000; // 0 to 9999
            const uint32_t c = value % 10000; // 0 to 9999
            
            const uint32_t d1 = (b / 100) << 1;
            const uint32_t d2 = (b % 100) << 1;
            
            const uint32_t d3 = (c / 100) << 1;
            const uint32_t d4 = (c % 100) << 1;
            
            *buffer++ = DLookupTable<2>::data[d1];
            *buffer++ = DLookupTable<2>::data[d1 + 1];
            *buffer++ = DLookupTable<2>::data[d2];
            *buffer++ = DLookupTable<2>::data[d2 + 1];
            *buffer++ = DLookupTable<2>::data[d3];
            *buffer++ = DLookupTable<2>::data[d3 + 1];
            *buffer++ = DLookupTable<2>::data[d4];
            *buffer++ = DLookupTable<2>::data[d4 + 1];
        }
        *buffer++ = '\0';
    }
    
    void u32toa_branchlut(uint32_t value, char* buffer) {
        if (value < 10000) {
            const uint32_t d1 = (value / 100) << 1;
            const uint32_t d2 = (value % 100) << 1;
            
            if (value >= 1000)
                *buffer++ = DLookupTable<2>::data[d1];
            if (value >= 100)
                *buffer++ = DLookupTable<2>::data[d1 + 1];
            if (value >= 10)
                *buffer++ = DLookupTable<2>::data[d2];
            *buffer++ = DLookupTable<2>::data[d2 + 1];
        }
        else if (value < 100000000) {
            // value = bbbbcccc
            const uint32_t b = value / 10000;
            const uint32_t c = value % 10000;
            
            const uint32_t d1 = (b / 100) << 1;
            const uint32_t d2 = (b % 100) << 1;
            
            const uint32_t d3 = (c / 100) << 1;
            const uint32_t d4 = (c % 100) << 1;
            
            if (value >= 10000000)
                *buffer++ = DLookupTable<2>::data[d1];
            if (value >= 1000000)
                *buffer++ = DLookupTable<2>::data[d1 + 1];
            if (value >= 100000)
                *buffer++ = DLookupTable<2>::data[d2];
            *buffer++ = DLookupTable<2>::data[d2 + 1];
            
            *buffer++ = DLookupTable<2>::data[d3];
            *buffer++ = DLookupTable<2>::data[d3 + 1];
            *buffer++ = DLookupTable<2>::data[d4];
            *buffer++ = DLookupTable<2>::data[d4 + 1];
        }
        else {
            // value = aabbbbcccc in decimal
            
            const uint32_t a = value / 100000000; // 1 to 42
            value %= 100000000;
            
            if (a >= 10) {
                const unsigned i = a << 1;
                *buffer++ = DLookupTable<2>::data[i];
                *buffer++ = DLookupTable<2>::data[i + 1];
            }
            else
                *buffer++ = '0' + static_cast<char>(a);
    
            const uint32_t b = value / 10000; // 0 to 9999
            const uint32_t c = value % 10000; // 0 to 9999
            
            const uint32_t d1 = (b / 100) << 1;
            const uint32_t d2 = (b % 100) << 1;
            
            const uint32_t d3 = (c / 100) << 1;
            const uint32_t d4 = (c % 100) << 1;
            
            *buffer++ = DLookupTable<2>::data[d1];
            *buffer++ = DLookupTable<2>::data[d1 + 1];
            *buffer++ = DLookupTable<2>::data[d2];
            *buffer++ = DLookupTable<2>::data[d2 + 1];
            *buffer++ = DLookupTable<2>::data[d3];
            *buffer++ = DLookupTable<2>::data[d3 + 1];
            *buffer++ = DLookupTable<2>::data[d4];
            *buffer++ = DLookupTable<2>::data[d4 + 1];
        }
        *buffer++ = '\0';
    }
    
    void i32toa_branchlut(int32_t value, char* buffer) {
        uint32_t u = static_cast<uint32_t>(value);
        if (value < 0) {
            *buffer++ = '-';
            u = ~u + 1;
        }
    
        u32toa_branchlut(u, buffer);
    }
    
    void u64toa_branchlut(uint64_t value, char* buffer) {
        if (value < 100000000) {
            uint32_t v = static_cast<uint32_t>(value);
            if (v < 10000) {
                const uint32_t d1 = (v / 100) << 1;
                const uint32_t d2 = (v % 100) << 1;
                
                if (v >= 1000)
                    *buffer++ = DLookupTable<2>::data[d1];
                if (v >= 100)
                    *buffer++ = DLookupTable<2>::data[d1 + 1];
                if (v >= 10)
                    *buffer++ = DLookupTable<2>::data[d2];
                *buffer++ = DLookupTable<2>::data[d2 + 1];
            }
            else {
                // value = bbbbcccc
                const uint32_t b = v / 10000;
                const uint32_t c = v % 10000;
                
                const uint32_t d1 = (b / 100) << 1;
                const uint32_t d2 = (b % 100) << 1;
                
                const uint32_t d3 = (c / 100) << 1;
                const uint32_t d4 = (c % 100) << 1;
                
                if (value >= 10000000)
                    *buffer++ = DLookupTable<2>::data[d1];
                if (value >= 1000000)
                    *buffer++ = DLookupTable<2>::data[d1 + 1];
                if (value >= 100000)
                    *buffer++ = DLookupTable<2>::data[d2];
                *buffer++ = DLookupTable<2>::data[d2 + 1];
                
                *buffer++ = DLookupTable<2>::data[d3];
                *buffer++ = DLookupTable<2>::data[d3 + 1];
                *buffer++ = DLookupTable<2>::data[d4];
                *buffer++ = DLookupTable<2>::data[d4 + 1];
            }
        }
        else if (value < 10000000000000000) {
            const uint32_t v0 = static_cast<uint32_t>(value / 100000000);
            const uint32_t v1 = static_cast<uint32_t>(value % 100000000);
            
            const uint32_t b0 = v0 / 10000;
            const uint32_t c0 = v0 % 10000;
            
            const uint32_t d1 = (b0 / 100) << 1;
            const uint32_t d2 = (b0 % 100) << 1;
            
            const uint32_t d3 = (c0 / 100) << 1;
            const uint32_t d4 = (c0 % 100) << 1;
    
            const uint32_t b1 = v1 / 10000;
            const uint32_t c1 = v1 % 10000;
            
            const uint32_t d5 = (b1 / 100) << 1;
            const uint32_t d6 = (b1 % 100) << 1;
            
            const uint32_t d7 = (c1 / 100) << 1;
            const uint32_t d8 = (c1 % 100) << 1;
    
            if (value >= 1000000000000000)
                *buffer++ = DLookupTable<2>::data[d1];
            if (value >= 100000000000000)
                *buffer++ = DLookupTable<2>::data[d1 + 1];
            if (value >= 10000000000000)
                *buffer++ = DLookupTable<2>::data[d2];
            if (value >= 1000000000000)
                *buffer++ = DLookupTable<2>::data[d2 + 1];
            if (value >= 100000000000)
                *buffer++ = DLookupTable<2>::data[d3];
            if (value >= 10000000000)
                *buffer++ = DLookupTable<2>::data[d3 + 1];
            if (value >= 1000000000)
                *buffer++ = DLookupTable<2>::data[d4];
            if (value >= 100000000)
                *buffer++ = DLookupTable<2>::data[d4 + 1];
            
            *buffer++ = DLookupTable<2>::data[d5];
            *buffer++ = DLookupTable<2>::data[d5 + 1];
            *buffer++ = DLookupTable<2>::data[d6];
            *buffer++ = DLookupTable<2>::data[d6 + 1];
            *buffer++ = DLookupTable<2>::data[d7];
            *buffer++ = DLookupTable<2>::data[d7 + 1];
            *buffer++ = DLookupTable<2>::data[d8];
            *buffer++ = DLookupTable<2>::data[d8 + 1];
        }
        else {
            const uint32_t a = static_cast<uint32_t>(value / 10000000000000000); // 1 to 1844
            value %= 10000000000000000;
            
            if (a < 10)
                *buffer++ = '0' + static_cast<char>(a);
            else if (a < 100) {
                const uint32_t i = a << 1;
                *buffer++ = DLookupTable<2>::data[i];
                *buffer++ = DLookupTable<2>::data[i + 1];
            }
            else if (a < 1000) {
                *buffer++ = '0' + static_cast<char>(a / 100);
                
                const uint32_t i = (a % 100) << 1;
                *buffer++ = DLookupTable<2>::data[i];
                *buffer++ = DLookupTable<2>::data[i + 1];
            }
            else {
                const uint32_t i = (a / 100) << 1;
                const uint32_t j = (a % 100) << 1;
                *buffer++ = DLookupTable<2>::data[i];
                *buffer++ = DLookupTable<2>::data[i + 1];
                *buffer++ = DLookupTable<2>::data[j];
                *buffer++ = DLookupTable<2>::data[j + 1];
            }
            
            const uint32_t v0 = static_cast<uint32_t>(value / 100000000);
            const uint32_t v1 = static_cast<uint32_t>(value % 100000000);
            
            const uint32_t b0 = v0 / 10000;
            const uint32_t c0 = v0 % 10000;
            
            const uint32_t d1 = (b0 / 100) << 1;
            const uint32_t d2 = (b0 % 100) << 1;
            
            const uint32_t d3 = (c0 / 100) << 1;
            const uint32_t d4 = (c0 % 100) << 1;
            
            const uint32_t b1 = v1 / 10000;
            const uint32_t c1 = v1 % 10000;
            
            const uint32_t d5 = (b1 / 100) << 1;
            const uint32_t d6 = (b1 % 100) << 1;
            
            const uint32_t d7 = (c1 / 100) << 1;
            const uint32_t d8 = (c1 % 100) << 1;
            
            *buffer++ = DLookupTable<2>::data[d1];
            *buffer++ = DLookupTable<2>::data[d1 + 1];
            *buffer++ = DLookupTable<2>::data[d2];
            *buffer++ = DLookupTable<2>::data[d2 + 1];
            *buffer++ = DLookupTable<2>::data[d3];
            *buffer++ = DLookupTable<2>::data[d3 + 1];
            *buffer++ = DLookupTable<2>::data[d4];
            *buffer++ = DLookupTable<2>::data[d4 + 1];
            *buffer++ = DLookupTable<2>::data[d5];
            *buffer++ = DLookupTable<2>::data[d5 + 1];
            *buffer++ = DLookupTable<2>::data[d6];
            *buffer++ = DLookupTable<2>::data[d6 + 1];
            *buffer++ = DLookupTable<2>::data[d7];
            *buffer++ = DLookupTable<2>::data[d7 + 1];
            *buffer++ = DLookupTable<2>::data[d8];
            *buffer++ = DLookupTable<2>::data[d8 + 1];
        }
        
        *buffer = '\0';
    }
    
    void i64toa_branchlut(int64_t value, char* buffer) {
        uint64_t u = static_cast<uint64_t>(value);
        if (value < 0) {
            *buffer++ = '-';
            u = ~u + 1;
        }
    
        u64toa_branchlut(u, buffer);
    }

}

