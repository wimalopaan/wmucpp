template<int> // Allg. template: Template-Definition
struct X {
    static inline constexpr int x = 0;
};

template<>
struct X<1> { // Vollständige Spezialisierung: Template-Definition
    static inline constexpr int x = 1;
};

//template<>
//constexpr int X<1>::x; // überflüssig, da schon eine vollst. Spezialisierung vorliegt, warnung bei g++, error bei clang++

//template<>
//constexpr int X<1>::x{}; // unsinnig (ill-formed), da schon eine vollst. Spezialisierung als Definition vorliegt, dies wäre dann die zweite Definition. error bei g++, error bei clang++

int main() {   
    const int * volatile p = & X<1>::x; // ok: gcc-7.2.1/gcc-8; nok: clang++-5.0
    return *p;
}


//template<int>
//struct X {
//    static constexpr int x = 0;
//    static constexpr int y = 1;
//};

////template<>
////constexpr int X<1>::x = 2; // should be ok, but gives duplicate initialization error
////template<>
////constexpr int X<1>::y; // should be ill-formed (constexpr must be initialized), but gives undefined reference

//int main() {   
//    return X<1>::x + X<1>::y;
//}
