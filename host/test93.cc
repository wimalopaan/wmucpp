template<auto N>
struct A {
    constexpr auto size() const {
        return N;
    }  
};

template<typename T>
constexpr void foo1(const T& a) {
    constexpr auto s = a.size(); // Why error here?
    return s;
}
template<typename T>
constexpr auto foo2(const T& a) {
    return a.size(); // Why OK here
}
int main() {
    A<10> x1;
//    constexpr auto s1 = foo1(x1);
    constexpr auto s2 = foo2(x1);
}
