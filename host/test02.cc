#include <cstddef>
#include <array>
#include <algorithm>
#include <iostream>
#include <util/meta.h>

template<typename ItemType, std::size_t rows, std::size_t columns>
class grid {
    std::array<float, rows * columns> elements{};
public:
    template<typename... T>
    constexpr grid(const T&... vv) : elements{float(vv)...} {
        static_assert(sizeof...(T) == (rows * columns));
    }

    template<typename ...T>
    constexpr auto operator()(T... rv) {
        static_assert(sizeof...(T) == columns);
        auto n = [&]<auto... II>(std::index_sequence<II...>){
                grid<ItemType, rows + 1, columns> n{elements[II]..., rv...};
                return n;
        }(std::make_index_sequence<rows * columns>{});
        return n;
    }
};

template<typename... RR>
grid(RR... rr) -> grid<Meta::front<Meta::List<RR...>>, 1, sizeof...(RR)>;

int main() {
    constexpr auto g = grid(1.0, 2.0)(3, 4)(5, 6);
    
    constexpr grid<double, 2, 3> g2{1, 2, 3, 4, 5, 6};
    
    
    std::array<std::array<int, 3>, 3> a{{{1, 2, 3}, {1, 7, 3}, {1, 2, 3}}};
    
    std::cout << a[1][1] << std::endl;
}
