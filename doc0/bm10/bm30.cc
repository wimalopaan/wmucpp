#include <stdint.h>
#include <stddef.h>
#include <array>

struct MbedSPI {
    void transfer(const void* buf, size_t count) {}
    
    template<auto N> void transfer(char (&a)[N]) {
        a[0] = 1;
//        std::integral_constant<size_t, N>::_;
    }
    
    template<auto N>
    void transfer(const std::array<std::byte, N>&) {}
};


int main() {
    char a[3];
    MbedSPI spi;
    
    spi.transfer(&a, 3);
    spi.transfer(a);
    
    std::array<std::byte, 3> a1;
    spi.transfer(a1);
    
//    decltype(a)::_;
//    decltype(&a)::_;
//    decltype(&a[0])::_;
    
    static_assert(&a[0] == a);
//    std::integral_constant<char (*)[3], &a>::_;
    
    return (int)(&a[0] + 1);
}
