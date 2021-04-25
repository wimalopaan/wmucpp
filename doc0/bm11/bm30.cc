#include <cassert>
#include <cstdint>
#include <type_traits>

//#define SIMPLE

// for macro lovers
#define VOLOP(x) _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wvolatile\"") \
    x; \
    _Pragma("GCC diagnostic pop")


#ifdef SIMPLE

struct add{};
struct sub{};
struct mul{};
struct div{};

// tagging overload
template<typename T>
void volOp(volatile T& v, const T& m, const add) {
    v = v + m;
}
template<typename T>
void volOp(volatile T& v, const T& m, const sub) {
    v = v - m;
}

// transform partial function specialisation (not possible) to constexpr-if
template<typename Op, typename T>
void volOp(T& v, const std::remove_volatile_t<T>& m) {
    if constexpr(std::is_same_v<Op, add>) {
        v = v + m;
    }
    else if constexpr(std::is_same_v<Op, sub>) {
        v = v - m;
    }
    else {
        static_assert(std::false_v<Op>);
    }
}

#else

struct add{
    template<typename T>
    T operator()(volatile T& a, const T& b) const {
        return a + b;
    }
};
struct sub{
    template<typename T>
    T operator()(volatile T& a, const T& b) const {
        return a - b;
    }    
};

template<typename T, typename Op>
requires std::is_volatile_v<T>
void volOp(T& v, const std::remove_volatile_t<T>& m, const Op op) {
    v = op(v, m);
}

// transform partial function specialisation (not possible) to constexpr-if
template<typename Op, typename T>
requires std::is_volatile_v<T>
void volOp(T& v, const std::remove_volatile_t<T>& m) {
    if constexpr(std::is_same_v<Op, add>) {
        v = v + m;
    }
    else if constexpr(std::is_same_v<Op, sub>) {
        v = v - m;
    }
    else {
        static_assert(std::false_v<Op>, "operation not implemented");
    }
}
#endif

volatile uint8_t a{}; 

int main() {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
    a += 2; // warning ignored
#pragma GCC diagnostic pop
            
    VOLOP(a += 1);
    
    // tagging
    volOp(a, uint8_t{1}, add{}); // should not work if a is not volatile (SIMPLE ignores that)
    volOp(a, uint8_t{1}, sub{});
    volOp(a, uint8_t{1}, []<typename T>(volatile T& a, const T& b){return a * b;});
    
    // explicit instanciation
    volOp<add>(a, uint8_t{2});
    volOp<sub>(a, uint8_t{2});
//    volOp<mul>(a, uint8_t{2});
 
    while(true);
}
