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

int main() {
    B<char> b;
    auto x = asA(b);
}

