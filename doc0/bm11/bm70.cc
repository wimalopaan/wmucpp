//#define NDEBUG

#include <std/compare>
#include <etl/ranged.h>
#include <cstdint>
#include <type_traits>

#if 0
struct A {
    uint8_t m1{0};
};

volatile uint8_t v;

template<typename T>
void f(const T& x) __attribute__((noinline));

template<typename T>
void f(const T& x) {
    if constexpr(std::is_same<T, A>::value) {
        v = x.m1;
    }
    else {
        v = x;
    }
}

uint8_t n1;
A n2;

etl::uint_ranged<uint8_t, 0, 10> n3;

int main() {
    f(n1);
    f(n2);
    f(n3);
}
#endif

#if 1
volatile uint8_t r;
volatile uint8_t o;

struct FSM {
    enum class State : uint8_t {A, B, C};
    static void f() __attribute__((noinline)){
        switch(mState) {
        case State::A:
            o = 10;
            break;
        case State::B:
            o = 11;
            break;
        case State::C:
            o = 12;
            break;
        default:
    //        std::unreachable();
            break;
        }
    }
    
    static void g() __attribute__((noinline)){
        static State mState2{State::A};
        switch(mState2) {
        case State::A:
            o = 10;
            break;
        case State::B:
            o = 11;
            break;
        case State::C:
            o = 12;
            break;
        default:
            std::unreachable();
            break;
        }
    }
private:
    inline static State mState{State::A}; // still modifyable via explicit template instantiation
};

enum class State : uint8_t {A, B, C};

void g(const State s) {
    switch(s) {
    case State::A:
        o = 10;
        break;
    case State::B:
        o = 11;
        break;
    case State::C:
        o = 12;
        break;
    default:
//        std::unreachable();
        break;
    }
}

template<typename T>
void f(const T& x) __attribute__((noinline));

template<typename T>
void f(const T& v) {
    switch(v) {
    case 10:
        o = v;
        break;
    case 11:
        o = v + 1;
        break;
    case 12:
        o = v + 2;
        break;
    default:
        //            break;
        std::unreachable();
    }
}

template<typename T>
void h(const etl::uint_ranged<T, 10, 12>& v) __attribute__((noinline));

template<typename T>
void h(const etl::uint_ranged<T, 10, 12>& v) {
    switch(v) {
    case 10:
        o = v;
        break;
    case 11:
        o = v + 1;
        break;
    case 12:
        o = v + 2;
        break;
//    default:
//        //            break;
//        std::unreachable();
    }
}

int main() {
    while(true) {
        etl::uint_ranged<uint8_t, 10, 12> a{r, etl::RangeCheck<false>{}};
//        etl::uint_ranged<uint8_t, 10, 12> a{r};
//        uint8_t a{r};
        f(a);
        
        h(a);
        
        g(State::A);
        g(State{0x0f});
        
        FSM::f();
        FSM::g();
    }
}
#endif

