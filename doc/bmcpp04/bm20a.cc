// Test für guard variable 

#include <cstdint>

struct A {
    inline constexpr A(uint8_t v) : m{v} {} // without constexpr it should not compile, but does anymay
    auto m1() const {
        return m;
    }
private:
     uint8_t m{0};
};

template<typename T>
struct X {
    static auto foo() {
        return m.m1();
    }
    constinit inline static T m{2}; // requires constexpr ctor
};
int main() {
    return X<A>::foo();    
}

