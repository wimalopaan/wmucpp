#include <cstdint>
#include <cstdlib>
#include <type_traits>
#if __has_include("compare")
# include <compare> 
#endif

template<auto V> struct A{};

#if (__GNUC__ <= 9)
struct B {
    constexpr explicit B(int a) : value{a} {}
//    inline constexpr bool operator==(const B& rhs) const {
//        return value == rhs.value;
//    }
private:
    const int value{0};
};
#endif

#if (__GNUC__ > 9)
struct B {
    constexpr explicit B(int a) : value{a} {}
    
//    inline constexpr auto operator<=>(const B& rhs) const = default;

//    inline constexpr auto operator<=>(int rhs) const {
//        return value <=> rhs;
//    };
    
    // to be structural all members have to be public and structural as well
    const int value{0};
};
#endif

int main() {
    A<B{42}> a1;
    A<B{42}> a2;
    
    constexpr B b1{0x815};
    B b2{0x816};

    A<b1> a3;
    
#if (__GNUC__ > 9)     
    return b1 <= b2;
    return b1 <  b2;
    return b1 >= b2;
    return b1 >  b2;
    return b1 != b2;

    return b1 <= 1;
    return 1 <= b1;
#endif
    return b1 == b2;
}

