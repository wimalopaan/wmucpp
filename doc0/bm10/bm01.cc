#include <cstdint>
#include <cstddef>
#include <limits>
#include <algorithm>

#if 0
constexpr uint16_t sum(const uint8_t* const array, const size_t length) {
    uint16_t sum{};
    for(size_t index{}; index <= length; ++index) {
        sum += array[index];
    }
    return sum;
}

constexpr auto test1 = []{
    uint8_t a[1]{};
    const auto s = sum(a, std::size(a));
    return s;
}();

constexpr int* foo(){ // trivial -> warnung
    int l;
    return &l;
}
constexpr auto test2 = []{
    return *foo();
}();
constexpr auto test5 = []{
    int z = 0, *p = &z;
     *p += z++;
    return *p;
}();


constexpr auto test3 = []{
    int* p{};
    {   
        int x{};
        p = &x;
    }
    return *p; // should be UB, but isn't
}(); // IIFE

constexpr auto test4 = []{
    int x = std::numeric_limits<int>::min();
    int y = -x;
    return y;
}(); // gives UB
#endif

int main() {
    int x = std::numeric_limits<int>::min();
    int y = -x;
    
}
