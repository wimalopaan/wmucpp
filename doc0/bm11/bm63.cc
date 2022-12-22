//struct A {
//    char e0;
//};
//struct B {
//    char e0;
//};

//template<typename T1, typename T2>
//union U {
//    T1 a;
//    T2 b;
//};

#include <cstdint>
#include <array>

template<typename T, size_t N>
struct Base {
    std::array<T, N> e;
    const T& operator[](size_t i) const {
        return e[i];
    }
};

template<typename T>
struct A : Base<T, 3> {
    
};

template<typename T>
struct B : Base<T, 3> {
    
};


int main() {
    const A<char> a{1, 2, 3};
    const B<char> b{4, 5, 6};
    
    return a[0] + b[0];
        
    
//    constexpr U<A, B> u{.a = {.e0 = 1}};
//    constexpr char t1 = u.b.e0;    
}
