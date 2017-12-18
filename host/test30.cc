#include <cstddef>
#include <type_traits>

namespace detail {
    template<size_t N>
    struct factorial_impl {
        typedef std::integral_constant<size_t, N * factorial_impl<N - 1>::type::value> type;
    };
    template<>
    struct factorial_impl<0> {
        typedef std::integral_constant<size_t, 1> type;
    };
}

template<size_t N>
using factorial = typename detail::factorial_impl<N>::type;

static_assert(factorial<1>::value == 1);
static_assert(factorial<4>::value == 24);

int main() {}
