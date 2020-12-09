# include <cstdint>
# include <cstring>
# include <cstddef>
# include <bit>
# include <array>

using to_t   = std::array<int16_t, 2>;

int main() {
    constexpr std::byte from1[sizeof(to_t)]{};
    constexpr auto v1 = std::bit_cast<to_t>(from1);    
    return v1[0];
}

