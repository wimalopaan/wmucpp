#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>

constexpr uint16_t sum(const uint8_t* const array, const size_t length) {
    uint16_t sum{};
    for(size_t index{}; index <= length; ++index) {
        sum += array[index];
    }
    return sum;
}

auto test1 = []{
    uint8_t a[1]{};
    const auto s = sum(a, std::size(a));
    return s;
};

constexpr int* foo(){ // trivial -> warnung
    int l;
    return &l;
}
auto test2 = []{
    return *foo();
};

auto test5 = []{
    int z = 0, *p = &z;
     *p += z++;
    return *p;
};

auto test3 = []{
    int* p{};
    {   
        int x{};
        p = &x;
    }
    return *p; // should be UB, but isn't
}; 

auto test4 = []{
    int x = std::numeric_limits<int>::min();
    int y = -x;
    return y;
}; // gives UB

int main() {
    //constexpr auto t1 = test4(); // ok, ubsan: ok
    /*constexpr*/ auto t2 = test3(); // nok, ubsan: nok
    /*constexpr */auto t3 = test5(); // nok, ubsan: nok
//    /*constexpr */auto t4 = test1(); // ok, ubsan: ok
//    constexpr auto t5 = test2(); // ok, ubsan: ok
}
