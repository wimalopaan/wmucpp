#include <mcu/avr.h>

#include <utility>
#include <array>

struct I {
    inline virtual constexpr uint16_t get() const = 0;
    inline virtual void set(uint8_t) = 0;
    inline virtual void toggle(uint8_t v) = 0;
};

struct A : public I {
    inline constexpr A(uint8_t v) : value{v} {}
    inline constexpr uint16_t get() const override {
        return value;
    }
    inline void set(uint8_t v) override {
        PORTA.OUT = v;
    }
    inline void toggle(uint8_t v) {
        PORTA.OUTTGL = v;
    }
    uint8_t value;
};
struct B : public I {
    inline constexpr B(uint16_t v) : value{v} {}
    inline constexpr uint16_t get() const override {
        return value;
    }
    inline void set(uint8_t v) override {
        PORTB.OUT = v;
    }
    inline void toggle(uint8_t v) {
        PORTB.OUTTGL = v;
    }
    uint16_t value;
};

namespace  {
    A x1{32};
    B x2{42};
    A x3{52};
    
//    float z;
}

int main() {
    std::array<I*, 6> p{&x1, &x2, &x3, &x1, &x2, &x3};
//    std::array<const I*, 3> p{&x1, &x2, &x3};
    
    p[0] = &x3;
    
    uint16_t x = 10;
    
//    x = p[0]->get();
//    return x;

    while(true) {
        for(const auto& i : p) {
            x += i->get();
            i->set(x);
            i->toggle(0x01);
        }
    }
    return x;
}
