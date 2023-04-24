//#define NDEBUG

#include <stdint.h>

volatile uint8_t o;

enum State {A, B, C};
/*static */enum State mState = A;

static void f() __attribute__((noinline));
static void f(){
    switch(mState) {
    case A:
        o = 10;
        break;
    case B:   
        o = 11;
        break;
    case C:
        o = 12;
        break;
    }
}
static void g() __attribute__((noinline));
static void g(){
    switch(mState) {
    case A:
        o = 10;
        break;
    case B:
        o = 11;
        break;
    case C:
        o = 12;
        break;
    default:
        __builtin_unreachable();
        break;
    }
}
static void h() __attribute__((noinline));
static void h() {
    switch(mState) {
    case A:
        o = 10;
        break;
    case B:
        o = 11;
        break;
    case C:
        o = 12;
        break;
    default:
        break;
    }
}
static void foo() __attribute__((noinline));
static void foo() {
    mState = 42; // von -fstrict-enums nicht erfasst
}
int main() {
    while(true) {
        f();
        g();
        h();
        
        foo();
    }
}

