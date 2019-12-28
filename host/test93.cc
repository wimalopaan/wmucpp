//#include <compare> 

template<auto V>
struct A {};

struct B {
    constexpr B(int a) : value{a} {}
//    inline constexpr auto operator<=>(const B& rhs) const = default;
private:
    int value{0};
};

int main() {
//    A<B{}> t1;
//    constexpr B b{0};
//    A<b> t2;        
}

