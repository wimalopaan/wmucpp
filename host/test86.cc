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
#if __has_include("new")
# include <new>
#else
# warning "local new"
inline constexpr void* operator new(std::size_t, void* p) {
    return p;
}
#endif

#if __has_include("type_traits")
# include <type_traits>
#endif

#if __has_include("bit")
# include <bit>
#endif

#ifndef __cpp_lib_is_constant_evaluated
# warning "local traits"
namespace std {
    inline constexpr bool is_constant_evaluated() {
        return __builtin_is_constant_evaluated();
    }
}
#endif

// using constexpr renders function inline
#define USE_CONSTEXPR
// using consteval make UB visible
//#define USE_CONSTEVAL

//#define USE_ASM
#define NO_UB

#ifdef USE_CONSTEVAL
# define CONSTEXPR consteval
#else 
# ifdef USE_CONSTEXPR
#  define CONSTEXPR constexpr
# else 
#  define CONSTEXPR
# endif
#endif
#ifdef USE_CONSTEXPR 
# define CONSTEXPR1 constexpr
#else
# define CONSTEXPR1
#endif

struct S {
    constexpr S() {
        if (!std::is_constant_evaluated()) {
#ifdef USE_ASM
            asm(";S()");
#endif
        }
    }
    uint8_t m{42};
};

struct V {
    constexpr V() {
        if (!std::is_constant_evaluated()) {
#ifdef USE_ASM
            asm(";V()");
#endif
        }
    }
    uint8_t m{43};    
};

typedef S from_type;
typedef V to_type;

union U {
    constexpr U(){}
    from_type d{};
    to_type i;
};

CONSTEXPR to_type test1(const from_type d) { // violates constexpr because of UB
#ifdef USE_ASM
    asm(";test1()>");
#endif
    U u;
    u.d = d; // activate member d;
#ifdef NO_UB
    u.d.~from_type();
    new (&u.i) to_type;
#endif
#ifdef USE_ASM
    asm(";test1()<");
#endif
    return u.i; // read from inactive member
}

CONSTEXPR1 to_type test2(const from_type d) { // violates constexpr because of std::memcpy (use std::bit_cast)
#ifdef USE_ASM
    asm(";test2()>");
#endif
    to_type r;
    std::memcpy(&r, &d, (sizeof(from_type) > sizeof(to_type) ? sizeof(to_type): sizeof(from_type)));
#ifdef USE_ASM
    asm(";test2()<");
#endif
    return r;
}

CONSTEXPR1 to_type test3(const from_type d) { // violates constexpr because of std::memcpy (use std::bit_cast)
#ifdef USE_ASM
    asm(";test3()>");
#endif
    to_type r;
    std::bit_cast<to_type>(d);
#ifdef USE_ASM
    asm(";test3()<");
#endif
    return r;
}

auto foo() {
    constexpr from_type x;
    return test2(x);    
}
int main() {
    from_type x;
    return foo().m + test3(x).m;    
}

static_assert(std::is_trivially_copyable<double>::value);
static_assert(std::is_trivially_copyable<S>::value);
static_assert(std::is_trivially_copyable<V>::value);
