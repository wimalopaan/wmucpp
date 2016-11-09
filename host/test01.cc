#include <iostream>

struct A {
    static constexpr const uint8_t value = 1;
    static void foo() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
    static void foo(int) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }

};

struct B {
    static constexpr const uint8_t value = 2;
    static void foo() {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }
    static void foo(int) {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
    }

};

struct C {
    static constexpr const uint8_t value = 42;
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
//        std::cout << __PRETTY_FUNCTION__ << std::endl;
        (PP::foo(),...);
    }
    static void foo(int i) {
//        std::cout << __PRETTY_FUNCTION__ << std::endl;
        (PP::foo(i),...);
    }
};

template<typename... PP>
struct ForEachCheckN {
    template<int N, typename P, typename... PPP>
    struct F {
        static void foo(int i) {
//            std::cout << __PRETTY_FUNCTION__ << std::endl;
            P::foo(i);
            F<N - 1, PPP..., void>::foo(i);
        }
    };
    template<typename P, typename... PPP>
    struct F<0, P, PPP...> {
        static void foo(int) {
//            std::cout << __PRETTY_FUNCTION__ << std::endl;
        }
    };

    static void foo(int i) {
//        std::cout << __PRETTY_FUNCTION__ << std::endl;
        F<sizeof...(PP), PP...>::foo(i);
    }
};

template<typename... PP>
struct ForEachCheck {
    template<typename P, typename... PPP>
    struct F {
        static void foo(int i) {
            if (P::value == i) {
                P::foo(i);
            }
            F<PPP..., void>::foo(i);
        }
    };
    template<typename... PPP>
    struct F<void, PPP...> {
        static void foo(int) {}
    };

    static void foo(int i) {
        F<PP...>::foo(i);
    }
};

int main() {
    ForEach<A, B, C>::foo();
    ForEach<A, B, C>::foo(42);

    ForEachCheck<A, B, C>::foo(42);
    ForEachCheck<A>::foo(42);

}

