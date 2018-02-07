#include <cstdint>
#include <cstddef>
#include <array>

namespace detail {
    template<size_t... II, typename A, typename B>
    inline void copy(A& a, const B& b, std::index_sequence<II...>) {
        ((a[II] = b[II]),...);
    }
}

template<typename A, typename B>
void copy(A& a, const B& b) {
    detail::copy(a, b, std::make_index_sequence<a.size>{});
}

volatile std::array<char, 6> a{'a', 'b', 'c', '\0'};
//volatile std::array<char, 6> b{'a', 'b', 'c', '\0'};

int main() {
    std::array<char, 6> b{'a', 'b', 'c', '\0'};
    copy(b, a);
    return b[0] + b[1] + b[2] + b[3] + b[4] + b[5];
}
