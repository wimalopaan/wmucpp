#include <array>
#include <algorithm>

struct P {
    /*static */bool operator()(uint8_t e) {
        return e == 3;
    }
    static bool f(uint8_t e) {
        return e == 3;
    }
};


//struct myfunction_args {
//  int x;
//  int bla;
//  const char* blup;
//};

//void myfunction_p(struct myfunction_args x) {
    
//}

//#define myfunction(...) myfunction_p((struct myfunction_args){__VA_ARGS__})


int main() {
//    myfunction(.x = 1, .x= 2, .blup="Hello World!");
            
    std::array<uint8_t, 5> a{1, 2, 3, 4, 5};

    P p;
    
    auto fit = std::find_if(std::begin(a), std::end(a), p);
//    auto fit = std::find_if(std::begin(a), std::end(a), P::f);
    
    return *fit;
}
