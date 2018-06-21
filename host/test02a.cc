#include <cstddef>
#include <array>
#include <algorithm>
#include <iostream>
#include <cassert>

// little bit of TMP (see below: deduction guide)
namespace Meta {
    template<typename... T> struct List;
    namespace detail {
        template<typename L> struct front;
        template<typename F, typename... T, template<typename...> typename L>
        struct front<L<F, T...>> {
            typedef F type;
        };
    }
    template<typename L>
    using front = typename detail::front<L>::type;
}

template<typename ItemType, std::size_t rows, std::size_t columns>
class grid {
    std::array<std::array<ItemType, columns>, rows> elements{};
    
    friend class grid<ItemType, rows - 1, columns>;
    
    template<auto... RR, typename... VV> 
    constexpr auto add_row(std::index_sequence<RR...>, const VV&... rv) const {
        std::array<ItemType, columns> nextRow{ItemType(rv)...};
        grid<ItemType, rows + 1, columns> n{(*this)[RR]..., nextRow};
        return n;
    }
    template<typename... AA>
    constexpr grid(const std::array<ItemType, columns>& a, const AA&... aa) : elements{a, aa...} {
        static_assert(sizeof...(AA) == (rows - 1), "wrong number of rows");
        static_assert((std::is_same_v<ItemType, typename AA::value_type> && ...));
    }
public:
    typedef ItemType value_type;
    typedef std::integral_constant<size_t, rows> row_size_type;
    typedef std::integral_constant<size_t, columns> column_size_type;

    template<typename... VV>
    constexpr grid(const VV&... vv) : elements{{ItemType(vv)...}} {
        static_assert(sizeof...(VV) == columns, "wrong number of columns");
    }
    
    template<typename... VV>
    constexpr auto operator()(const VV&... rv) const {
        static_assert(sizeof...(VV) == columns, "wrong number of values to form a complete row");
//        static_assert((std::is_same_v<ItemType, VV> && ...), "use always the same type"); 
        return [&]<auto... RR>(std::index_sequence<RR...>) { // C++20, refactor to function-template if C++17
                std::array<ItemType, columns> nextRow{ItemType(rv)...};
                grid<ItemType, rows + 1, columns> n{(*this)[RR]..., nextRow};
                return n;
        }(std::make_index_sequence<rows>{});

//        return add_row(std::make_index_sequence<rows>{}, rv...);
    }
    
    constexpr auto& operator[](size_t row) const {
        assert(row < rows);
        return elements[row];
    }
};

// user-defined deduction-guide
template<typename... RR>
grid(RR... rr) -> grid<Meta::front<Meta::List<RR...>>, 1, sizeof...(RR)>;

int main() {
    constexpr auto g1 = grid(1.0, 2.0)(3, 4)(5, 6); // grid<double, 3, 2>
    constexpr auto g2 = grid(1, 2)(3, 4)(5, 6); // grid<int, 3, 2>
}
