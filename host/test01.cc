#include <cstddef>
#include <array>
#include <algorithm>
#include <initializer_list>

template<typename... T>
struct R {
    constexpr R(T... vv) {};
};

template<std::size_t rows, std::size_t columns>
class grid {
    std::array<std::array<float, columns>, rows> elements{};
public:
//    template<typename... RR>
//    constexpr grid(RR... r) {}

        template<typename... T>
    constexpr grid(const T&... vv) noexcept {}
    
//    constexpr grid(std::initializer_list<std::initializer_list<float>>) {}
};




template<auto R, auto C>
struct Row {
    template<typename... T>
    constexpr Row(T... rv) : elements{rv...}{}  
        
    template<typename ...T>
    constexpr auto operator()(T... rv) {
        auto n = [&]<auto... II>(std::index_sequence<II...>){
                Row<R + 1, C> n{elements[II]..., rv...};
                return n;
        }(std::make_index_sequence<R * C>{});
        return n;
    }
    constexpr auto operator()() {
        auto g = [&]<auto... II>(std::index_sequence<II...>) {
                grid<R, C> g{elements[II]...};
                return g;
        }(std::make_index_sequence<R * C>{});
        return g;
    }
    std::array<float, R * C> elements{};
};

template<typename... T>
constexpr auto make_grid(T... c) {
    return Row<1, sizeof...(T)>(c...);  
}

template<typename... RR>
grid(RR... rr) -> grid<sizeof...(RR), sizeof...(RR)>;

//constexpr auto make_grid(std::initializer_list<std::initializer_list<float>> il) {
//     return grid<2,2>{};
//}


int main() {
    constexpr auto g = make_grid(1, 2)(3, 4)();
    
//    constexpr grid g{{1, 2}, {3, 4}};
    
//    auto l = []{
//        float v[2][2] = {{1, 2}, {3, 4}};
//    };
    
//    constexpr auto g = make_grid{{1, 2}, {3, 4}};

//    constexpr auto g = grid{R{1, 2}, R{3, 4}};    
}
