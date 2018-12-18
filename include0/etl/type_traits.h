#pragma once

#include <cstdint>
#include <limits>

namespace etl {
    namespace detail {
        template<auto Bits>
        struct typeForBits {
            using type = typename std::conditional<(Bits <= 8), uint8_t, 
                             typename std::conditional<(Bits <= 16), uint16_t, 
                                 typename std::conditional<(Bits <= 32), uint32_t,
                                      typename std::conditional<(Bits <= 64), uint64_t, void>::type>::type>::type>::type;
        };
        template<auto V>
        struct typeForValue {
            using type = typename std::conditional<(V > std::numeric_limits<uint32_t>::max()), uint64_t, 
                                  typename std::conditional<(V > std::numeric_limits<uint16_t>::max()), uint32_t,
                                    typename std::conditional<(V > std::numeric_limits<uint8_t>::max()), uint16_t, uint8_t>::type>::type>::type;
        };
    
        
    }
    
    
    template<typename E, typename = std::enable_if_t<std::enable_bitmask_operators_v<E>>>
    inline constexpr bool
    toBool(E v) {
        return static_cast<bool>(v);        
    }
    
    template<typename T>
    constexpr uint8_t numberOfBits() {
        return sizeof(T) * 8;
    }

    template<uint64_t Bits>
    using typeForBits_t = typename detail::typeForBits<Bits>::type;  
    
    template<auto V>
    using typeForValue_t = typename detail::typeForValue<V>::type;
    
    
    
    template<template<typename...> typename A, template<typename...> typename B>
    struct is_same_template : std::false_type {};
    template<template<typename...> typename T>
    struct is_same_template<T, T> : std::true_type {};
    
}
