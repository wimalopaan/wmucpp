#include <array>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string_view>
#include <cassert>



#if 0
template<typename C, size_t L>
struct LString {
    consteval LString(const C* const p) : ptr{p} {} // immdiate-function: only for string-literals
    consteval LString(const C (&a)[L] ) {} // immdiate-function: only for string-literals
    constexpr C operator[](const size_t i) const {
        assert(i < L);
        return *(ptr + i);
    }
private:
    const C* ptr{};
};

template<typename C, size_t L>
struct LString2 {
    consteval LString2(const C (&a)[L]) : ptr{a} {}   // NOK
//    consteval LString2(const C (&a)[L]) : ptr{} {}  // OK
    const C* ptr{};
};

template<LString2 U>
consteval auto operator"" _sl(){
    return U;
}
namespace detail {
    template<typename C>
    struct size;
    template<typename E, size_t N>
    struct size<E[N]> {
        static inline constexpr size_t value = N;
    };
    template<typename C, size_t N>
    struct size<LString<C, N>> {
        static inline constexpr size_t value = N;            
    };
    
    template<typename T, typename... TT>
    struct front {
        using type = T;
    };
    
    template<typename C>
    struct item_type;
    template<typename E, size_t N>
    struct item_type<E[N]> {
        using type = E;   
    };
    template<typename E, size_t N>
    struct item_type<LString<E, N>> {
        using type = E;   
    };        
}    
template<typename C>
constexpr auto size_v = detail::size<C>::value;

template<typename C>
using item_type_t = detail::item_type<C>::type;

template<typename... TT>
using front = detail::front<TT...>::type;

struct AlwaysNullTerminated {
    static inline constexpr size_t addToLength{1};
};
struct MaybeNullTerminated {
    static inline constexpr size_t addToLength{0};    
};

template<typename Term, typename... C>
constexpr auto concat(Term, const C&...  c) {
    constexpr size_t length = (size_v<C> + ...) + Term::addToLength;
    
    using item_type = std::remove_cvref_t<item_type_t<front<C...>>>;
    static_assert((std::is_same_v<item_type, std::remove_cvref_t<item_type_t<C>>> && ...));
    
    using result_t = std::array<item_type, length>;
    result_t result;
   
    size_t index{0};
    auto insert = [&]<typename CC>(const CC& c) {
                  for(size_t i = 0; i < size_v<CC>; ++i, ++index) {
                    if (c[i] != '\0') {
                        assert(index < length);
                        result[index] = c[i];
                    }
                    else {
                        break;
                    }
                }
            };
     (insert(c), ...);        
     if (index < length) {
          result[index] = '\0';
     }
     return result;
}    

template<typename E, size_t N>
consteval auto lstring(const E(&a)[N]) { // immediate function at compile-time
    static_assert(N >= 1);
    return LString<E, N - 1>(a);
}
#define LStr(s) lstring(s)

#endif

#if 0
template<typename C, size_t L>
struct LString {
    constexpr LString(const C* const p) : ptr{p} {} // should be consteval
    constexpr C operator[](const size_t i) const {
        return *(ptr + i);
    }
private:
    const C* const ptr{};
};

template<typename C>
struct size;
template<typename E, size_t N>
struct size<E[N]> {
    static constexpr size_t value = N;
};
template<typename C, size_t N>
struct size<LString<C, N>> {
    static constexpr size_t value = N;            
};

template<typename C, typename... CC>
struct totalsize {
    static constexpr size_t value = size<C>::value + totalsize<CC...>::value;  
};
template<typename C>
struct totalsize<C> {
    static constexpr size_t value = size<C>::value;  
};

template<typename E, size_t N>
constexpr LString<E, N - 1> lstring(const E(&a)[N]) { // should be consteval
    static_assert(N >= 1);
    return LString<E, N - 1>(a);
}

#define LStr(s) lstring(s)

template<typename O, typename C>
void insert(O& out, size_t& index, const C& c) {
    for(size_t i = 0; i < size<C>::value; ++i, ++index) {
        if (c[i] != '\0') {
            out[index] = c[i];
        }
        else {
            break;
        }
    }    
}
template<typename O, typename C, typename... CC>
void insert(O& out, size_t& index, const C& c, const CC&... cc) {
    insert(out, index, c);    
    insert(out, index, cc...);    
}

template<typename... C>
constexpr auto concat(const C& ... c) -> std::array<char, totalsize<C...>::value> {
    constexpr size_t length = totalsize<C...>::value;
    std::array<char, length> result; 
    size_t index{0};
    
    insert(result, index, c...);
    
    if (index < length) {
        result[index] = '\0';
    }    
    return result;
}

template<typename S, typename C>
S& operator<<(S& stream, const C& a) {
    for(const auto c : a) {
        if (c != '\0') {
            stream << c;
        }
        else {
            break;
        }
    }
    return stream;
}
#endif

template<size_t... L>
struct NList {};

template<size_t... LL>
consteval auto generate(const char (&...ss)[LL]) {
    constexpr size_t lmax = std::max({LL...}) - 1;
    std::array<char, 1 + (sizeof...(ss) * lmax)> r{lmax};
    size_t position = 1;
    auto insert = [&](auto&& s){
        for(size_t i = 0; i < lmax; ++i) {
            if (i < sizeof(s)) {
                r[position++] = s[i];
            }
            else {
                r[position++] = '\0';
            }
        }
    };    
    (insert(ss), ...); 
    
    return r;        
}

template<size_t... LL>
consteval auto generate2(const char (&...ss)[LL]) {
    std::tuple<std::array<char, LL - 1>...> r;
    [&]<size_t... I, size_t... L>(std::index_sequence<I...>, const char (&...s)[L]) {
        auto insert = [&]<size_t N>(std::array<char, N>& a, auto&& v) {
                      for(size_t i{0}; auto& e : a) {
                            e = v[i++];
                        }
        };
        (insert(std::get<I>(r), s), ...);
    }(std::make_index_sequence<sizeof...(ss)>{}, ss...); 
    return r;
}

namespace detail {
    template<typename T, typename F, size_t... I>
    constexpr void all(const T& tuple, const F& f, std::index_sequence<I...>) {
        (f(std::get<I>(tuple)), ...);
    }    
}
template<typename... T, typename F>
constexpr void visit(const std::tuple<T...>& tuple, const F& f) {
    detail::all(tuple, f, std::make_index_sequence<sizeof...(T)>{});
}

template<size_t N, size_t L>
constexpr std::string_view to_stringview(const std::array<char, L>& tr) {
    return std::string_view{&tr[1 + N*tr[0]], &tr[1 + (N+1)*tr[0]]};
}

constexpr auto STR_X = generate("abc", "def", "xyz");
constexpr auto STR_Y = generate2("abc", "def2", "xyz33");

int main() {
//    decltype(STR_Y)::_;
    
    visit(STR_Y, []<size_t L>(const std::array<char, L>& a) {
        std::cout << std::string_view{&a[0], &a[L]} << '\n';        
    });
    
    std::cout << to_stringview<0>(STR_X) << '\n';
    
    return 0;
    
//    char text[3] = "xy";
////    text[2] = 'z';
    
//    auto x1 = concat(AlwaysNullTerminated{}, "abcd", text, "def");
//    auto x2 = concat(MaybeNullTerminated{}, LStr("abcd"), text, LStr("def"));
//    auto x3 = concat(AlwaysNullTerminated{}, LStr("abcd"), text, LStr("def"));
    
//    std::cout << std::size(x1) << " : " << x1 << '\n';
//    std::cout << std::size(x2) << " : " << x2 << '\n';
//    std::cout << std::size(x3) << " : " << x3 << '\n';
    
////    auto z = "xxx"_sl;
    
}
