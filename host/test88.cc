#include <functional>
#include <iostream>
#include <concepts>

//class Bildschirm{
//  public:
//    void f() const {
//        button(*this);
//    }
//    void g() const {
//        std::cout << __PRETTY_FUNCTION__ << '\n';        
//    }
//    void onF(std::invocable<Bildschirm> auto const f) {
//        button = f;
//    }
//private:
//    std::function<void(const Bildschirm&)> button;
//};


//struct A {
//    constexpr void g(A& o, int v) {
//        o = A{.x = v, .f = [this]{
////            std::cout << "f" << this->x << '\n';
//        }};
//    }
//    int x{0};
//    std::function<void()> f;
//    ~A() {
////        std::cout << "dtor" << x << '\n';
//    }
//};

//consteval void test() {
//    A a;
//    a.g(a, 2);
//    a.f();
//}

template<size_t N>
struct Bildschirm {
    constexpr static void onButtonPress(auto f) {
        f();
    }
    
};

template<size_t N>
struct Usart {
    constexpr static void put(const std::byte) {}
};

using usart = Usart<0>;
using b1 = Bildschirm<0>;
//using b1 = Bildschirm<0, usart>;

int main() {   
    b1::onButtonPress([]{
    });
    
//    test();
//    Bildschirm Bild1;
    
//    Bild1.onF([](const Bildschirm& b){
//        std::cout << __PRETTY_FUNCTION__ << '\n';        
//        b.g();
//    });
    
//    Bild1.f();
}
