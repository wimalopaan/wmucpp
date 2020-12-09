# include <cstdint>
# include <cstring>
# include <cstddef>
# include <bit>
# include <array>

using to_t   = std::array<int16_t, 2>;

namespace {
    volatile uint8_t from2[sizeof(to_t)]{1, 2, 3, 4};
}
int main() {
    constexpr std::byte from1[sizeof(to_t)]{0x01_B, 0x02_B, 0x03_B, 0x04_B};
    constexpr auto v1 = std::bit_cast<to_t>(from1); // constexpr bit_cast now working
    
    const auto v2 = std::bit_cast<to_t>(from2); // same as below

    const auto v3 = []{
        to_t data;
        std::memcpy(&data, (void*)&from2, sizeof(to_t)); // not usable in constexpr-context
        return data;
    }();

    return v1[0] + v2[0] + v3[0];
}

