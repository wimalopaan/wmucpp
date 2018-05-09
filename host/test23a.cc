#include <cstdint>

template<typename T>
constexpr int bar(const T& x) { // NOK
//constexpr int bar(T x) { // OK
    return x - 1;
}

//template<typename T>
//int foo(const T& l) {
//    constexpr auto x = l();
//    auto y = []{return bar(x);}; // if ref-capture is used, the above bar(const T&) is NOK, why? 
    
//    if constexpr(x <= 0) {
//        return 42;
//    }
//    else {
//        return foo(y);
//    }
//}

//auto l2 = [] {
//    return 3;
//};

constexpr auto z = 1;
int main() {
    auto y = [&]{
        return bar(z);
    };
    constexpr auto z2 = y();
     
//    foo(l2);
}

