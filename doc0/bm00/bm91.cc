#include <utility>
#include <array>

struct MCU {
    struct P1 {
        static inline volatile std::byte r1;
        static inline volatile std::byte r2;
    };
    struct P2 {
        static inline volatile std::byte r1;
        static inline volatile std::byte r2;
    };
};

struct I {
    inline virtual constexpr std::byte get() const = 0;
    inline virtual void set(std::byte) const = 0;
    inline virtual void toggle(std::byte v) const = 0;
    inline virtual void store() = 0;
};

struct A : public I {
    inline constexpr explicit A(std::byte v) : value{v} {}
    inline constexpr std::byte get() const override {
        return value;
    }
    inline void set(std::byte v) const override {
        MCU::P1::r1 = v;
    }
    inline void toggle(std::byte v) const override {
        MCU::P1::r1 |= v;
    }
    inline void store() override {
        value = MCU::P1::r2;
    }
    std::byte value{};
};

struct B : public I {
    inline constexpr explicit B(std::byte v) : value{v} {}
    inline constexpr std::byte get() const override {
        return value;
    }
    inline void set(std::byte v) const override {
        MCU::P2::r1 = v;
    }
    inline void toggle(std::byte v) const override {
        MCU::P2::r1 |= v;
    }
    inline void store() override {
        value = MCU::P2::r2;
    }
    std::byte value{};
};

namespace  {
    A x1{32_B};
    B x2{42_B};
    A x3{52_B};
    B x4{62_B};   
}

int main() {
    std::array<I*, 4> p{&x1, &x2, &x3, nullptr};
    p[0] = &x1;
    while(true) {
        std::byte x{};
        for(auto&& i : p) {
            if (i) {
                i->store();
                x |= i->get();
            }
        }
        x1.set(x);
    }
}
