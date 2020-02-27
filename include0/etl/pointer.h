#pragma once

#include <type_traits>
#include <cassert>

template<typename T>
requires(!std::is_same_v<T, nullptr_t>)
struct not_null {
    constexpr not_null(const T p) : ptr{p} {
        assert(ptr != nullptr);
    }
private:
    T ptr;    
};
