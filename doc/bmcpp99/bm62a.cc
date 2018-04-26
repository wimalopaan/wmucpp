#include <cstdint>
#include <cstddef>
#include <utility>

#include "mcu/avr8.h"
#include "mcu/register.h"

#include "util/meta.h"
#include "util/type_traits.h"

using flagRegister = AVR::RegisterFlags<typename DefaultMcuType::GPIOR, 0, uint8_t>;

struct Registrar {
    uint8_t nextBit() {
        return ++bitCounter;
    }
private:
    uint8_t bitCounter{};
};

template<typename T>
struct A {
    A(Registrar& r) : bit{r.nextBit()}{
        flagRegister::get() |= ((uint8_t)1 << bit);
    }
    T mData;
    const uint8_t bit{};
};

template<typename T>
struct B {
    B(Registrar& r) : bit{r.nextBit()} {
        if (flagRegister::get() & (uint8_t)((uint8_t)1 << bit)) {
            flagRegister::get() &= ~((uint8_t)1 << bit);
        }
    }
    T mData;
    const uint8_t bit{};
};

Registrar reg;

A<uint8_t>  a(reg);
B<uint16_t> b(reg);

int main() {}
