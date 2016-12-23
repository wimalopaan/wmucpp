#include <stdint.h>

// Heise Artikel
template <uintptr_t port, uint8_t bit>
class GpioOut {
public:
    GpioOut(bool initState) {
        set(initState);
        *ddrReg() |= 1 << bit;
    }

    void set(bool state) {
        if (state) {
            *portReg() |= 1 << bit;
        } else {
            *portReg() &= ~(1 <<bit);
        }
    }
private:
    constexpr uint8_t volatile *portReg() {
        return reinterpret_cast<uint8_t volatile *>(port);
    }
    constexpr uint8_t volatile *ddrReg() {
        return portReg() - 1;
    }
};

int main() {
    GpioOut<0x55, 0> p(false);
    
    while(true) {}
}
