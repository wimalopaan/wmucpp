#include <avr/io.h>

// enum-class ist int, wenn nicht spezifiziert
// enum-class mit underlying-type verhindert -fstrict-enums, da alle Werte des underlying-types möglich sind
// unscoped-enums werden mit -fshort-enums optimiert zu kleinstem möglichen Datentyp
// unscoped-enums mit 2er-Potenz an Werten werden mit -fstrict-enums optimiert -> kein std::unreachable notwendig
//    es werden nur die enum-Werte auf Gleichheit geprüft bei -fstrict-enums
//    ohne -fstrict-enums gibt es Werte des underlying ohne Action
//    allerdings: mit std::unreachable gibt es leicht kleineren Code wegen Zusammenfassung
// unscoped-enums ohne 2er-Potenz an Werten werden mit -fstrict-enums nicht optimiert -> std::unreachable
//    hier bleiben ungenutzte Werte in dem hypothetischen Integer übrig, d.h. es gibt trotzdem im Assembler Pfad ohne Aktion
//    damit gleiches Verhalten wie ohne -fstrict-enums


#ifdef NDEBUG
# undef NDEBUG
#endif

#define NDEBUG
#define P2
//#define MISS

// NDEBUG    P2      strict-enums |  optimal     size
//---------------------------------------------------------------
//    -       -            -      |    -          314 
//    -       -            x      |    -          314
//    -       x            -      |    -          322
//    -       x            x      |    x          318
//    x       -            -      |    x          310
//    x       -            x      |    x          310       
//    x       x            -      |    x          316
//    x       x            x      |    x          318


#include <stdint.h>

volatile uint8_t o;

struct FSM {
#ifdef P2
    enum State {A, B, C, D};
#else
    enum State {A, B, C};
#endif
    static void f() __attribute__((noinline)) {
        switch(mState) {
        case State::A:
            o = 10;
            break;
        case State::B:
            o = 11;
            break;
#if defined(P2) || !defined(MISS)
        case State::C:
            o = 12;
            break;
#endif
#ifdef P2
# ifndef MISS
        case State::D:
            o = 13;
            break;
# endif
#endif
#ifdef NDEBUG
        default:
            __builtin_unreachable();
            break;
#endif
        }
    }
    static void set(State s) {
        mState = s;
    }
private:
    inline static State mState{State::A};
};

int main() {
    while(true) {
        FSM::f();
        //        FSM::set(FSM::A);
    }
}
