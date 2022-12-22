#include <assert.h>

// in temp.h
enum types {TypeA, TypeB, TypeC};

struct Union {
    enum types d;
    char buffer[1024];
};

// in temp.c
struct Union unionLikeBuffer;

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
    struct A mA;
    struct B mB;
    struct C mC;
};

union Data data;

int a(union Data d) {
    return d.mA.a;
}

int main() {
    return a(data);
}
