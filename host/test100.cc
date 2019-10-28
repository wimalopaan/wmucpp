#include <concepts>

//struct A {
//    inline constexpr A(int v = 0) : m{v} {}
//    int m1() const {
//        return m;
//    }
//private:
//    int m = 0;
//};

//namespace AN {
//    template<typename T>
//    struct X {
//        static int foo() {
//            return m.m1();
//        }
//        constinit inline static T m{1};  
//    };
    
//    template<typename T>
//    using X = X<T>;
    
//}
//int main() {
//    AN::X<A>::m = 2;
//    return AN::X<A>::foo();    
//}

template<typename T>
concept Foo = requires {
    {T::value} -> std::same_as<bool>;
};

struct B {
    constexpr B() {}    
};

struct A {
    constexpr A(int x) 
        : ptr(new int(x))
    {
    }
    constexpr ~A() {
        delete ptr;
    }
    int* ptr = nullptr;
//    constexpr static inline B b2{}; 
//    constinit static inline B b1{}; 
};

struct X {
    constinit inline static A a{1};
};


int main() {
}
