#include <mcu/avr.h>
#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <algorithm>
#include <initializer_list>
#include <etl/stringbuffer.h>
#include <etl/algorithm.h>

template<typename T, typename I = T>
static inline consteval auto indexes() {
    return []<auto... II>(std::index_sequence<II...>){
        return std::array<I, sizeof...(II)>{I(II)...};
    }(std::make_index_sequence<T::Upper + 1>{});
}

template<typename T, typename I = T::value_type>
struct range {
    struct Iterator {
        inline constexpr bool operator!=(const Iterator& o) const {
            return i != o.i;
        }
        inline constexpr void operator++() {
            ++i;
        }
        inline constexpr I operator*() const {
            return i;
        }
        I i{};
    };
    inline constexpr Iterator begin() const {
        return Iterator{0};
    }    
    inline constexpr Iterator end() const {
        return Iterator{T::Upper + 1};
    }    
};


volatile uint8_t v;

int main() {
    using index_type = etl::uint_ranged<uint8_t, 0, 8>;
    
//    for(const auto& i : indexes<index_type>()) {
//        v = i;
//    }

    for(auto&& i : range<index_type>{}) {
        v = i;
    }

    for(uint8_t i{0}; i < index_type::Upper + 1; ++i) {
        v = i;
    }
    
//    for(const auto& i : {0, 1, 2, 3, 4, 5, 6, 7}) {
//        v = i;
//    }

}

