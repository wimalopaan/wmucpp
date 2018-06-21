#include <cstdint>
#include <cstddef>
#include <array>

#include "util/meta.h"

template<auto N>
struct Spi final {
    static void put(std::byte data) {
        sfr = data;
    }
private:
    inline static volatile std::byte sfr;
};

struct Ascii;
struct Binary;

template<auto Length>
struct ArithmeticSum {
    using value_type = std::conditional_t<(Length < 256), uint16_t, uint32_t>;

    value_type get() const {
        return sum;
    }
    void process(std::byte value) {
        sum += std::to_integer(value);
    }
    template<typename Device>
    void out() const {
        std::byte lower_sum = std::byte(sum);
        std::byte upper_sum = std::byte(sum>> 8);
        Device::put(lower_sum);
        Device::put(upper_sum);
        if constexpr(Length >= 256) {
            std::byte lower2_sum = std::byte(sum >> 16);
            std::byte upper2_sum = std::byte(sum >> 24);
            Device::put(lower2_sum);
            Device::put(upper2_sum);
        }
    }
private:
    value_type sum{};
};

template<auto Length>
struct BinarySum {
    using value_type = std::byte;
    value_type get() const {
        return sum;
    }
    void process(std::byte value) {
        sum ^= value;
    }
    template<typename Device>
    void out() const {
        Device::put(sum);
    }
private:
    value_type sum{};
};

template<typename Kind, typename Dev>
struct Protocol final {
    template<auto L>
    static constexpr void put(const std::array<std::byte, L>& data) {
        using checkermap = Meta::List<Meta::List<Ascii, ArithmeticSum<L>>, Meta::List<Binary, BinarySum<L>>>;
        Meta::map<Kind, checkermap> checker;
        for(const auto& v : data) {
            checker.process(v);
            Dev::put(v);
        }
        checker.template out<Dev>();
    }
};

using spi1  = Spi<0>;
using spi2  = Spi<1>;
using p1   = Protocol<Ascii, spi1>;
using p2   = Protocol<Binary, spi2>;

std::array<std::byte, 2> a1;
std::array<std::byte, 4> a2;
std::array<std::byte, 8> a3;
std::array<std::byte, 16> a4;

int main(){
    p1::put(a1);
    p1::put(a2);
    p2::put(a3);
    p2::put(a4);
}
