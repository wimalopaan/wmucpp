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
inline constexpr void* operator new(std::size_t, void* p) {
    return p;
}
#endif
#if __cpp_lib_is_constant_evaluated
# include <type_traits>
#else
namespace std {
    inline constexpr bool is_constant_evaluated() {
        return __builtin_is_constant_evaluated();
    }
}
#endif

// using constexpr renders function inline
//#define USE_CONSTEXPR
// using consteval make UB visible
#define USE_CONSTEVAL

//#define USE_STRUCT
//#define USE_MIXED
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
    constexpr S(uint8_t) {
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

#ifdef USE_STRUCT
typedef S from_type;
typedef V to_type;
#else
# ifdef USE_MIXED
typedef double from_type;
typedef int32_t to_type;
# else 
typedef char from_type;
typedef char to_type;
# endif
#endif
union U {
    constexpr U(){}
    from_type d;
    to_type i;
};

CONSTEXPR to_type test1(const from_type d) { // violates constexpr because of UB
#ifdef USE_ASM
    asm(";test1()>");
#endif
    U u;
    u.d = d; // activate member d;
#ifdef NO_UB
    u.d.~from_type(); // end-of-life
    new (&u.i) to_type(); // begin-of-life with same representation
#endif
#ifdef USE_ASM
    asm(";test1()<");
#endif
    return u.i; // read from inactive member (with UB) or newly active member (w/o UB)
}

CONSTEXPR1 to_type test2(const from_type d) { // violates constexpr because of std::memcpy (use std::bit_cast)
#ifdef USE_ASM
    asm(";test2()>");
#endif
    to_type r; // begin-of-life
    std::memcpy(&r, &d, (sizeof(from_type) > sizeof(to_type) ? sizeof(to_type): sizeof(from_type))); // copy-state (must be trivially-copyable)
#ifdef USE_ASM
    asm(";test2()<");
#endif
    return r;
}

auto foo() {
    constexpr from_type x{1};
//    return test1(x);    
}
int main() {
    from_type x{1};
#ifdef USE_STRUCT
    return foo().m + test2(x).m;    
#else
//    return foo() + test2(x);    
#endif
}
