#include <algorithm>
#include <iterator>
#include <iostream>
#include <array>
#include <vector>

template<typename T>
concept Container = requires(T c) {
                    std::begin(c);
                    std::end(c);
                    c[size_t{}];
                    };

auto median(auto values) { 
//auto median(Container auto values) { 
    std::nth_element(std::begin(values), std::begin(values) + std::size(values) / 2, std::end(values));
    return values[std::size(values) / 2];
}

auto sum(std::input_iterator auto first, std::input_iterator auto last) {
    auto r{*first};
    while(++first != last) {
        r += *first;
    }    
    return r;
}

struct X {
    float value{};
    int   m{};
};

namespace detail {
    template<typename It, typename F>
    struct Iter {
        using difference_type = ptrdiff_t;
        using value_type = decltype(F{}(*It{}));
        constexpr auto operator*() const {
            return f(*it);
        }
        constexpr Iter& operator++() {
            ++it;
            return *this;
        }
        constexpr Iter operator++(int) {
            Iter copy{*this};
            ++it;
            return copy;
        }
        constexpr bool operator!=(Iter rhs) const {
            return it != rhs.it;
        }
        It it;
        F f;
    };
}
constexpr auto adapt_begin(const auto& c, auto f) {
    return detail::Iter{std::begin(c), f};
}
constexpr auto adapt_end(const auto& c, auto f) {
    return detail::Iter{std::end(c), f};
}

int main() {
    const std::vector<X> values{{1.0f, 1}, {2.1f, 2}, {2.0f, 3}};
    
    std::vector<float> fv;
    std::transform(std::begin(values), std::end(values), std::back_inserter(fv),[](auto x){return x.value;});
    
    std::cout << median(fv) << '\n';    
    std::cout << sum(std::begin(fv), std::end(fv)) << '\n';
    
    auto la{[](auto x){return x.value;}};
    std::cout << sum(adapt_begin(values, la), adapt_end(values, la)) << '\n';
    
    float floats[]{1.0, 2.0, 3.0};
    
    std::cout << sum(std::begin(floats), std::end(floats)) << '\n';
    
//    auto x1{median(fv)};    
//    auto x2{sum(std::begin(fv), std::end(fv))};
    
//    auto la{[](auto x){return x.value;}};
    auto x3{sum(adapt_begin(values, la), adapt_end(values, la))};
    
    auto x4{sum(std::begin(floats), std::end(floats))};
    
    return x3 + x4;
    
}
