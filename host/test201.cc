#if __has_include("cstdint")
# include <cstdint>
#else 
# include <stdint.h>
#endif

#if __has_include("cstddef")
# include <cstddef>
#else
# include <stddef.h>
namespace std {
    enum class byte : uint8_t{};  // for the AVR people
}
#endif

struct S {uint8_t value;};
struct T {uint8_t value;};

template<typename A, typename B>
[[gnu::noinline]] 
auto test1(A& p1, B& p2) {
    if constexpr(requires{p1.value; p2.value;}){
        p1.value = 1; 
        p2.value = 2;
        return p1.value + p2.value;
    }
    else if constexpr(requires{*p1 = 0;}){
        *p1 = 1; 
        p2  = 2;
        return *p1 + p2;
    }
    else if constexpr(requires{p1 = A{1}; p2 = B{1};}){
        p1 = A{1};
        p2 = B{2};
        return uint8_t(p1) + uint8_t(p2);
    }
    else {
        static_assert(sizeof(A) == 0, "wrong instantiation");
    }
}

// test1() setzt x1 aktiv, dann wird x2 aktiv gesetzt, dann wir vom inactiven member gelesen -> UB und falsch (type-punning)
// test1<S,T> macht KEIN reread
uint8_t testA() {
    union {
        S x1;
        T x2;
    } u{};
    return test1(u.x1, u.x2);
}
// Ergebnis ist 3 -> falsch (type-punning)
// test1<S,T> macht KEIN reread
uint8_t testB() {
    S x1{};
    return test1(x1, reinterpret_cast<T&>(x1)); 
}
// Ergebnis ist 3 -> korrekt
// test1<S,T> macht KEIN reread
uint8_t testC() {
    S x1{};
    T x2{};
    return test1(x1, x2); 
}
// Ergebnis ist 4 -> korrekt
// test1<S,S> muss einen reread machen
uint8_t testD() {
    S x1{};
    return test1(x1, x1); 
}
// Ergebnis ist 4 -> korrekt
// test1<uint8_t, uint8_t> MUSS einen reread machen
uint8_t testE() {
    uint8_t x1;
    return test1(x1, x1); 
}
// Ergebnis ist 3 -> korrekt
// test1<uint8_t, uint16_t> MUSS einen reread machen, weil uint8_t ALLES aliasen kann -> unperformant
uint8_t testF() {
    uint8_t x1;
    uint16_t x2;
    return test1(x1, x2); 
}
// Ergebnis ist 3 -> korrekt
// test1<uint16_t, uint32_t> macht KEIN reread 
uint8_t testG() {
    uint16_t x1;
    uint32_t x2;
    return test1(x1, x2); 
}
// Ergebnis ist 3 -> korrekt
// test1<std::byte, uint16_t> MUSS einen reread machen, weil std::byte ALLES aliasen kann -> unperformant
uint8_t testH() {
    std::byte x1;
    uint16_t  x2;
    return test1(x1, x2); 
}

// Ergebnis ist 3 -> korrekt
// test1<Char, uint16_t> macht KEIN reread machen -> performant
uint8_t testI() {
    char8_t  x1;
    uint16_t x2;
    return test1(x1, x2); 
}

template<typename T, typename Tag>
struct NamedType { // structural type
    T value; // keine Invarianten
};

// Ergebnis ist 3 -> korrekt
// test1<width, height> macht KEIN reread machen -> performant
uint8_t testK() {
    using width  = NamedType<uint8_t, struct Length>;
    using height = NamedType<uint8_t, struct Width>;
    width  x1{};
    height x2{};
    return test1(x1, x2); 
}

// Ergebnis ist 3 -> korrekt
// test1<uint16_t*, uint16_t> MUSS reread machen -> unperformant
uint8_t testL() {
    uint16_t  v{};
    uint16_t* x1{&v};
    uint16_t  x2{};
    return test1(x1, x2); 
}

struct X {
    template<typename T>
    [[gnu::noinline]]
    uint8_t test1(const T& o) {
        if constexpr(requires{o.value;}) {
            value += o.value;
            return o.value;
        }
        else {
            value += o;
            return o;
        }
    }    
    uint8_t value{};
};

uint8_t testO() {
    X x1;
    X x2;
    return x1.test1(x2); // Aliasing wäre möglich
}
uint8_t testO1() {
    X x1;
    return x1.test1(x1); // Aliasing
}
uint8_t testP() {
    X x1;
    S x2;
    return x1.test1(x2); // kein Aliasing möglich
}
uint8_t testQ() {
    X    x1;
    char x2;
    return x1.test1(x2); // immer Aliasing
}

struct X1 {
//    operator int() const {return k;}
//    private:
    int k;
};

int fn(const X1& x, int& p) {
    int i = x.k;
    p = 2;
    return i + x.k;
}

int g() {
    X1 x{};
    return fn(x, x.k);
}
int main() {}
