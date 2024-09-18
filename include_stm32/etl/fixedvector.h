#pragma once

#include <algorithm>
#include <initializer_list>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <cassert>

namespace etl {    
    template<typename T, auto Capacity>
    class FixedVector final {
    public:
        using size_type = std::conditional_t<Capacity <= 255, uint8_t, uint16_t>;
        using value_type = T;
        inline static constexpr const size_type capacity = Capacity;
        
        template<typename... TT>
        requires((std::is_same_v<T, TT> && ...) && (sizeof...(TT) <= Capacity))
        constexpr FixedVector(const TT&... e) : data{e...}, mSize{sizeof...(TT)} {
        }
        
        inline constexpr T& operator[](size_type index) {
            assert(index < Capacity);
            return data[index];
        }
        inline constexpr const T& operator[](size_type index) const {
            assert(index < Capacity);
            return data[index];
        }
        inline constexpr const T* begin() const {
            return &data[0];
        }
        inline constexpr const T* end() const {
            return &data[mSize];
        }
        inline constexpr void reserve(size_type s) {
            assert(s <= Capacity);
            mSize = std::min(s, Capacity);
        }
        inline constexpr size_type size() const {
            return mSize;
        }
        inline constexpr void clear() {
            mSize = 0;
        }
        inline constexpr void push_back(const T& item) {
            assert(mSize < Capacity);
            data[mSize++] = item;
        }
        inline constexpr const T& back() const {
            assert(mSize > 0);
            return data[mSize - 1];
        }
        inline constexpr T& back() {
            assert(mSize > 0);
            return data[mSize - 1];
        }
        inline constexpr const T& front() const {
            assert(mSize > 0);
            return data[0];
        }
        inline constexpr T& front() {
            assert(mSize > 0);
            return data[0];
        }
    private:
        T data[Capacity];
        size_type mSize{0};
    };
}
