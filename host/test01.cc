#include <iostream>

struct A {
    static void foo() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
    static void foo(int) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }

};

struct B {
    static void foo() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
    static void foo(int) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }

};

struct C {
    static void foo() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
    static void foo(int) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
};

template<typename... PP>
struct ForEach {
    static void foo() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        (PP::foo(),...);
    }
    static void foo(int i) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        (PP::foo(i),...);
    }
};

template<typename... PP>
struct ForEachCheck {
    template<int N, typename P, typename... PPP>
    struct F {
        static void foo(int i) {
            std::cout << __PRETTY_FUNCTION__ << std::endl;
            P::foo(i);
            F<N - 1, PPP..., void>::foo(i);
        }
    };
    template<typename P, typename... PPP>
    struct F<0, P, PPP...> {
        static void foo(int) {
            std::cout << __PRETTY_FUNCTION__ << std::endl;
        }
    };

    static void foo(int i) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        F<sizeof...(PP), PP...>::foo(i);
    }
};


int main() {
    ForEach<A, B, C>::foo();
    ForEach<A, B, C>::foo(42);

    ForEachCheck<A, B, C>::foo(42);
    ForEachCheck<A>::foo(42);

}

