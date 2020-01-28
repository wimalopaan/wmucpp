#include <cstdint>
#include <cstdlib>
#include <type_traits>
#include <chrono>
#if __has_include("compare")
# include <compare> 
#endif

template<auto V> struct A{};

#if (__GNUC__ <= 9)
struct B {
    constexpr explicit B(int a) : value{a} {}
private:
    const int value{0};
};
#endif

#if (__GNUC__ > 9)
struct B {
    constexpr explicit B(int a) : value{a} {}
    constexpr auto operator<=>(const B&) const = default;
    // to be structural all members have to be public and structural as well
//private:
    const int value{0};
};
#endif

int main() {
    A<B{0x0042}> a1;
    A<B{0x0042}> a2;
    A<B{0x0815}> a3;

    static_assert(std::is_same_v<decltype(a1), decltype(a2)>);
    static_assert(!std::is_same_v<decltype(a1), decltype(a3)>);
    
    using namespace std::literals;
    using namespace std::chrono_literals;
    using namespace std::literals::chrono_literals;
    
//    A<1min> a4; // not possible
    
    auto t1 = 12s / 3;
    
//    auto t2 = 12d / 3;
//    decltype(t1)::_;    
    
//    auto t2 = std::chrono::(12);
    
}
