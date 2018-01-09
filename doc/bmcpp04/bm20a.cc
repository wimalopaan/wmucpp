
// Test für guard variable ... wird aber nicht erzeugt

struct A {
    volatile int m = 0;
};


template<typename T>
struct X {
    static int foo() {
        return m.m;
    }
    inline static T m;  
};

X<A> x;

int main() {
    return x.foo();    
}
