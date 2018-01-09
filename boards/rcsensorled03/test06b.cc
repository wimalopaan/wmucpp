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
    static T mTop; 
//    inline static T mTop; // guards
    //    inline static T mTop{}; 
};

template<typename T>
T B<T>::mTop; // guards

A a1; // no guards

int main() {
//    a1.foo();
    B<A>::foo();
//    B<C>::foo();
}
