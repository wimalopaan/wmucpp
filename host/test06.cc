#include <cstdint>
#include <array>
#include <algorithm>
#include <numeric>

namespace UC {
    volatile uint8_t portb;
}

#define PORTB UC::portb

template<size_t N>
using Iterations = std::integral_constant<size_t, N>;

template<typename> struct StaticFor;

template<size_t N>
struct StaticFor<std::integral_constant<size_t, N>> {
    static inline constexpr void call(const auto f) {
        call_impl(std::make_index_sequence<N>{}, f);
    }
private:
    template<size_t... II>
    static inline constexpr void call_impl(std::index_sequence<II...>, const auto f) {
        (f(II), ...);
    }
};

template<size_t N = 20>
class Setup {
    using MeinArray = std::array<uint16_t, N>;
    using MeinContainer = MeinArray;
    static inline constexpr auto a = []{
        MeinArray data;
        std::iota(std::begin(data), std::end(data), 0);
        return data;
    }();
    static inline constexpr auto b = []{
        MeinArray data;
        std::iota(std::begin(data), std::end(data), 0);
        return data;
    }();
    static inline constexpr auto c = []{
        MeinContainer data;
        std::transform(std::begin(a), std::end(a), std::begin(b), std::begin(data), 
                       [](const auto ai, const auto bi){return ai + bi;});
        return data;
    }();
public:
    static inline void init() {
        StaticFor<Iterations<20>>::call([](const auto i){
            PORTB = c[i];
        });
//        [&]<auto... II>(std::index_sequence<II...>) {
//            ((PORTB = c[II]), ...);
//        }(std::make_index_sequence<std::size(c)>{});
    }
};

int main() {
    Setup<>::init();
    
    UC::portb; 
}
