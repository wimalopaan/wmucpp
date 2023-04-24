#include <array>
#include <bit>

template<typename T>
struct A{
    T e0;
    T e1;
    T e2;
    //...
};
template<typename T>
struct B{
    T e0;
    T e1;
    T e2;
};

template<typename T>
A<T> asA(const B<T>& v){
    return std::bit_cast<A<T>>(v);
}

#define UDR *(reinterpret_cast<volatile char*>(0x0010));

int main() {
    
    UDR;
    
    B<char> b;
    auto x = asA(b);
}

