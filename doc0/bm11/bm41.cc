#include <array>


//static const int a1[300] = {
//  [ 2] = 123,
//  [13] = 456,
//  [75] = 789
//};

const auto a = []{
    std::array<int, 100> data;
    data[ 2] = 123;
    data[13] = 456;
    data[75] = 789;
    return data;
}();

volatile uint8_t r;

struct S {
    S() = default;
    S(const S&) {
        asm(";cctor");
    }
    uint8_t x = 42;
};

int main() {
//    const S s = S();
    
//    const auto s2 = []{
//        S data;
////        return S(); // RVO
//        return data; // no NRVO
//    }();

    auto s3 = []{
        std::array<S, 3> data;
        for(auto& x : data) {
            x.x = r;
        }
        return data;
    }();

    uint8_t sum = 0;
    for(const auto& x : s3) {
        sum += x.x;
    }
    
    return sum;
    
//    auto a = []{
//        std::array<int, 100> data;
//        data[ 2] = r;
//        data[13] = 456;
//        data[75] = 789;
//        return data;
//    }();
//    a[2] += r;
//    return a[2] + s2.x;
}
