#include <avr/io.h>

#undef _MMIO_BYTE
#define _MMIO_BYTE(x) (uintptr_t(x))

template<typename C>
typename C::value_type* getAddress() {
    return reinterpret_cast<typename C::value_type*>(C::address_value);
}

template<uintptr_t address>
struct Port {
    inline static constexpr uintptr_t address_value = address;
    typedef volatile uint8_t value_type;
    typedef Port type;
    
    template<auto Bit>
    static void set() {
        *getAddress<type>() |= (1 << Bit);
    }
};

using gp0 = Port<GPIOR0>;

int main() {
    gp0::set<1>();
}
