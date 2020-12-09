#if __has_include("cstdint")
# include <cstdint>
# else 
# include <stdint.h>
#endif
#if __has_include("cstring")
# include <cstring>
#else 
namespace std {
# include <string.h>
}
#endif

#if __has_include("variant")
# include <variant>
#endif

#if __has_include("bit") 
# include <bit>
#endif

// using constexpr renders function inline
//#define USE_CONSTEXPR
// using consteval make UB visible
//#define USE_CONSTEVAL
//#define USE_STRUCT 
//#define USE_MISMATCH

#ifdef USE_CONSTEVAL
# define CONSTEXPR consteval
#else 
# ifdef USE_CONSTEXPR
#  define CONSTEXPR constexpr
# else 
#  define CONSTEXPR
# endif
#endif

struct ToS {
    int a[4];
};
struct FromS {
    char b[sizeof(ToS)];
};

template<bool B, typename T, typename F> struct conditional {typedef F type;};
template<typename T, typename F> struct conditional<true, T, F> {typedef T type;};

#ifndef USE_STRUCT
typedef double from_type;

# ifdef USE_MISMATCH
typedef std::int64_t to_type;
# else
typedef typename conditional<(sizeof(from_type) == 8), std::uint64_t, 
                              typename conditional<(sizeof(from_type) == 4), std::uint32_t,
                              typename conditional<(sizeof(from_type) == 2), std::uint16_t, 
                              std::uint8_t>::type>::type>::type to_type;
# endif
#else 
typedef FromS from_type;
typedef ToS to_type;
#endif

union U {
    from_type d;
    to_type i;
};

CONSTEXPR to_type get1(const from_type d) { // violates constexpr because of UB
    U u;
    u.d = d; // activate member d;
    return u.i; // read from inactive member
}
CONSTEXPR to_type get2(const from_type d) { // violates constexpr because of std::memcpy (use std::bit_cast)
    to_type r;
    std::memcpy(&r, &d, (sizeof(from_type) > sizeof(to_type) ? sizeof(to_type): sizeof(from_type)));
    return r;
}

#if __cpp_lib_bit_cast 
    static_assert(sizeof(to_type) == sizeof(from_type));
//    static_assert(std::is_trivially_copyable<from_type>::value);
//    static_assert(std::is_trivial<to_type>::value);
CONSTEXPR to_type get3(const from_type d) {
    return std::bit_cast<to_type>(d);
}
#endif

CONSTEXPR to_type get4(const from_type d) {
    return *reinterpret_cast<const to_type*>(&d);
}

CONSTEXPR int test5() {
    int mask{};
    for(uint8_t i = 0; i < 17; ++i) {
        mask = 1 << i;
    }
    return mask;
}

CONSTEXPR int test6() {
    int x[1]{};
    x[10] = 0;
    return x[0];
}

struct S {uint8_t m;};
struct T {uint8_t m;};

template<typename A, typename B>
//[[gnu::noinline]] 
CONSTEXPR uint8_t test7(A& p1, B& p2) {
    p1.m = 1;
    p2.m = 2;
    return p1.m + p2.m;
}

CONSTEXPR uint8_t test8() {
    union {
        S x1;
        T x2;
    } u;
    return test7(u.x1, u.x2);
}

CONSTEXPR uint8_t test9() {
    S x1;
    return test7(x1, reinterpret_cast<T&>(x1)); // not constexpr
}
CONSTEXPR uint8_t test10() {
    S x1;
    T x2;
    return test7(x1, x2); 
}
CONSTEXPR uint8_t test11() {
    S x1;
    return test7(x1, x1); 
}

int main() {
    //    constexpr auto v1 = get1(1.1);
    //    constexpr auto v2 = get2(1.1);

#ifdef __cpp_lib_variant
    constexpr auto v1 = std::variant<int, double>{};
#endif
//    constexpr auto v2 = get4(1.1);
    int v3 = test5();
    int v6 = test6();
    int v7 = test8();
    
#ifdef USE_STRUCT
    return get1(FromS()).a[0] + get2(FromS()).a[1];
#else
    return get3(1.1);
//    return get1(1.1) + get2(1.1) + get4(1.1);
#endif
}
