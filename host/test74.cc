#include <cstdint>
#include <tuple>
#include <functional>

namespace Dry {
    template<typename Op, typename... T>
    struct op_t : private std::tuple<std::remove_volatile_t<std::remove_reference_t<T>>...> {
        friend Op;
        inline static constexpr size_t N = sizeof...(T);
        using base = std::tuple<std::remove_volatile_t<std::remove_reference_t<T>>...>;
        
        op_t(const T&... t) : base(t...) {}
        
        template <typename U>
        friend constexpr bool operator<(U&& u, op_t&& a) {
            return std::forward<op_t>(a) > std::forward<U>(u);            
        }
        template <typename U>
        friend inline constexpr bool operator>(U&& u, op_t&& a) {
            return std::forward<op_t>(a) < std::forward<U>(u);            
        }
        template <typename U>
        friend inline constexpr bool operator==(U&& u, op_t&& a) {
            return std::forward<op_t>(a) == std::forward<U>(u);            
        }
        template <typename U>
        inline constexpr bool operator<(U&& rhs) const {
            return Op::template apply<N>(std::forward<U>(rhs), std::greater{}, *this);
        }
        template <typename U>
        inline constexpr bool operator>(U&& rhs) const {
            return Op::template apply<N>(std::forward<U>(rhs), std::less{}, *this);
        }
        template <typename U>
        inline constexpr bool operator==(U&& rhs) const {
            return Op::template apply<N>(std::forward<U>(rhs), std::equal_to{}, *this);
        }
        template <typename U>
        friend inline constexpr bool operator!=(U&& u, op_t&& a) {
            return std::forward<op_t>(a) != std::forward<U>(u);            
        }
        template <typename U>
        inline constexpr bool operator!=(U&& rhs) const {
            return Op::template apply<N>(std::forward<U>(rhs), std::not_equal_to{}, *this);
        }
    };
    
    namespace detail {
        struct Or {
            template<auto N, typename U, typename F, typename T>
            inline static constexpr auto apply(const U rhs, const F& f, const T& t) {
                using base = typename T::base;
                return [&]<auto... II>(std::index_sequence<II...>){
                    return (f(rhs, std::get<II>(static_cast<base>(t))) || ...);
                }(std::make_index_sequence<N>{});
            };
        };
        struct And {
            template<auto N, typename U, typename F, typename T>
            inline static constexpr auto apply(const U rhs, const F& f, const T& t) {
                using base = typename T::base;
                return [&]<auto... II>(std::index_sequence<II...>){
                    return (f(rhs, std::get<II>(static_cast<base>(t))) && ...);
                }(std::make_index_sequence<N>{});
            };
        };
    }

    template<typename... T>
    struct any_of : op_t<detail::Or, T...>{
        any_of(const T&... t) : op_t<detail::Or, T...>{t...} 
        {}
    };
    template<typename... T> 
    struct each_of : op_t<detail::And, T...> {
        each_of(const T&... t) : op_t<detail::And, T...>{t...} {
        }    
    };
}
    
volatile uint8_t x{1};
uint8_t y{2};
uint8_t z{3};
volatile uint8_t v1{2};

//#define PLAIN

int main() {
    using namespace Dry;
    uint8_t r{};
#ifdef PLAIN
    if ((v1 < x) || (v1 < y) || (v1 < z)) {
        r += 1;
    }
    if ((x == v1) || (y == v1) || (z == v1)) {
        r += 2;
    }
    if ((v1 != x) || (v1 != y) || (v1 != z)) {
        r += 3;
    }
    if ((v1 != x)  && (v1 != z)) {
        r += 10;
    }
#else
    if (v1 < any_of{x, y, z}) {
        r += 1;
    }
    if (any_of{x, y, z} == v1) {
        r += 2;
    }
    if (v1 == any_of{x, y, z}) {
        r += 3;
    }
    if (v1 != each_of{x, z}) {
        r += 10;
    }
#endif
    return r;
}
