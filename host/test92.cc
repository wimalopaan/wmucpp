#include <iostream>
#include <cstdint>
#include <cassert>

class Float {
public:
    ~Float() {
        std::cout << __PRETTY_FUNCTION__ << '\n';
    };
    Float() {
        std::cout << __PRETTY_FUNCTION__ << '\n';        
    };
    Float (long) {
        std::cout << __PRETTY_FUNCTION__ << '\n';       
    };
    explicit Float(double) {
        std::cout << __PRETTY_FUNCTION__ << '\n';        
    };

    Float (const Float&) {
        std::cout << __PRETTY_FUNCTION__ << '\n';        
    };
    Float& operator=(const Float&) {
        std::cout << __PRETTY_FUNCTION__ << '\n';
        return *this;    
    }
    Float (Float&&) {
        std::cout << __PRETTY_FUNCTION__ << '\n';        
    };
    Float& operator= (Float&&) {
        std::cout << __PRETTY_FUNCTION__ << '\n';
        return *this;        
    }
};

auto foo(){
    Float f;
    double x = 3.1e10;
    f = x;
    return f;
}

int main() {
    foo();
}
