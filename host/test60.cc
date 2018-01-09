struct A {
    A() = default;
    void foo() {}
};
struct C {
    C() = default;
    void foo() {}
};

template<typename T>
struct B {
    static void foo() {
        mTop.foo();
    }
    inline static T mTop; // guards
};

A a1; // no guards

int main() {
//    a1.foo();
    B<A>::foo();
//    B<C>::foo();
}
