#include <iostream>
#include <type_traits>
#include <vector>
#include <cassert>

// generische Deklaration des RT nur so möglich
template<typename T1, typename T2>
auto sum1(const T1& t1, const T2& t2) -> decltype(t1 + t2){
    return t1 + t2;
}

// Vereinfachung durch type-inference, wenn übereinstimmende return-stmts
template<typename T1, typename T2>
auto sum2(const T1& t1, const T2& t2) {
    return t1 + t2;
}

// Deklaration nicht möglich, weil ohne Implementierung kein return-stmt
template<typename T1, typename T2>
auto sum3(const T1& t1, const T2& t2); // not possible

// (*) Einzige Möglichkeit, bei einer Deklaration den Typ generisch abzuleiten
template<typename T1, typename T2>
auto sum4(const T1& t1, const T2& t2) -> decltype(t1 + t2);

// trailing-RT machbar, aber nicht notwendig, weil nested-type in C
template<typename C>
C::value_type sum5(const C& c) {
    return {42};
}

// Achtung! trailing RT aus op[]
template<typename C>
auto sum6(const C& c) -> decltype(c[0]) { // c[0] -> int&
    if (c.empty()) {
        return {}; // temporary by-ref
    }
    return c[0];
}

template<typename> struct Abc{};

// keine trailung RT notwendig
template<typename T>
T sum7(const Abc<T>& x) {
    return {};
} 

//SFINAE ohne trailing RT
template<typename T>
std::enable_if_t<std::is_fundamental_v<T>, T>
sum8(const T& x) {
    return x;
}

// (*) SFINAE mit trailing RT
template<typename T>
auto sum9(const T& x) -> std::enable_if_t<std::is_fundamental_v<T>, T>{
    return x;
}

// SFINAE ersetzt durch Contraint
template<typename T> requires (std::is_fundamental_v<T>)
T sum10(const T& x) {
    return x;
}
// SFINAE ersetzt durch trailing constraint (notwendig, falls aus param-value abzuleiten)
template<typename T> 
T sum11(const T& x) requires (std::is_fundamental_v<T>) {
    return x;
}

int test1(){
    return 42;
}
double test2(){
    return 42;
}
auto test3() ->int(*)() {
    return test1;
}
// unleserlich
int (*foo1())() {
    return test1;
}
// (*) wesentlich besser zu lesen
// foo2() -> Zeiger auf Funktion double()
auto foo2() -> double(*)() {
    return test2;
}
// (*) wesentlich besser zu lesen
// foo3() -> Zeiger auf Funktion, die Zeiger auf Funktion int() liefert
auto foo3() -> auto(*)() -> auto(*)() -> int {
    return test3;
}



template<typename T>
auto foo4(T v) -> T(*)(){
    return test1;
}

template<typename T>
using T_func_void_t = T(*)();

template<typename T>
auto foo5(T v) -> T_func_void_t<T> {
    return test1;
}


int main() {
    auto x1{sum1(1, 2)};
    auto x2{sum2(1, 2)};
    
//    auto x3{sum3(1, 2)}; 
    auto x4{sum4(1, 2)};

    std::vector<int> v{1,2};    
    auto x5{sum5(v)};
    auto x6{sum6(v)};

    auto x7{sum7(Abc<int>{})};
    
    foo3()()();
    
    auto x20{foo5(int{})()};
    
    auto x8{sum8(1)};
//    auto x8{sum8(v)};
    auto x9{sum9(1.0)};
//    auto x9{sum9(v)};
    auto x10{sum10(1)};
//    auto x10{sum10(v)};
}

template<typename T1, typename T2>
auto sum3(const T1& t1, const T2& t2) {
    return t1 + t2;
}

template<typename T1, typename T2>
auto sum4(const T1& t1, const T2& t2) -> decltype(t1 + t2) {
    return t1 + t2;
}
