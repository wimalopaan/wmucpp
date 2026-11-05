#pragma once

#include <cstddef>
#include <cstdint>

namespace etl {
#if 1
    template<size_t len>
    struct StaticString {
        using data_t = std::array<char, len>; 
        const data_t data;
        consteval StaticString(const char (&s)[len]) : data {[&]{
            data_t d;
            std::copy(&s[0], &s[len], &d[0]);
            return d;
        }()}{}
    };
#else
    template<size_t Size>
    struct StaticString {
        char data[Size];
        constexpr size_t size() const {
            return Size;
        }
        consteval StaticString(const char (&s)[Size]){
            std::copy(&s[0], &s[Size], &data[0]);
        }
    };
#endif
    template<StaticString... SS>
    struct StringList {
        static inline constexpr size_t size() {
            return sizeof...(SS);
        }
        template<auto N>
        static inline const char* get() {
            return &SS...[N].data[0];
        }
    };
}
