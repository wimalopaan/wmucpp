#include <array>
#include <vector>
#include <iterator>
#include <iostream>

template<typename T>
void printRange( const T& begin, const T& end) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
}

int main() {
    std::array<int, 10> a1;
    int a2[10];
    std::vector<int> a3;
    
    printRange(std::begin(a1), std::end(a1));
    printRange(std::begin(a2), std::end(a2));
    printRange(std::begin(a3), std::end(a3));
    
}
