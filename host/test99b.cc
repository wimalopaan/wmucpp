#include <assert.h>

// in a.h
struct A {
    int a;
    int b;
};
// in b.h
struct B {
    float c;
    double d;
};
// in c.h
struct C {
    char text[80];
};

// in project.h
union Data {
public:
    int a() const {
        return mA.a;
    }
    void initA() { // begin life of mA
        mA = A{};
    }
private:
    A mA;
    B mB;
    C mC;
};

Data data;

int main() {
    // part1
    data.initA();
    
    return data.a();    
}
