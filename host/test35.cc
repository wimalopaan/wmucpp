#include <iostream>
#include <algorithm>
#include <array>
#include <utility>
#include <optional>
#include <experimental/array>

using std::experimental::make_array;

struct RightOpen;
struct Adjacent;

template<typename T, typename Type>
struct Interval {
    typedef T value_type;
    typedef Type kind_type;
    constexpr bool includes(const T& v) const {
        if constexpr(std::is_same_v<Type, RightOpen>) {
            return (v >= left) && (v < right);
        }
    }
    T left{};
    T right{};
};

template<typename Type, typename Construct, typename Limits>
constexpr auto make_intervals(const Limits& limits) {
    if constexpr(std::is_same_v<Construct, Adjacent>) {
        using item_type = typename Limits::value_type;
        std::array<Interval<item_type, Type>, limits.size() - 1> in;
        for(size_t n = 0; n < in.size(); ++n) {
            in[n].left = limits[n];
            in[n].right = limits[n + 1];
        }
        return in;
    }
    else {
        // ...
    }
}

template<typename T, typename Intervals>
std::optional<size_t> lookup(const T& value, const Intervals& intervals) {
    for(size_t n = 0; n < intervals.size(); ++n) {
        if (intervals[n].includes(value)) {
            return n;
        }
    }
    return {};
}

int main() {
    constexpr auto limits = make_array(0, 50, 100, 200, 400, 800);
    constexpr auto intervals = make_intervals<RightOpen, Adjacent>(limits);
    
    int value = 100;
    
    auto index = lookup(value, intervals);
    
    if (index) {
        std::cout << "Index: " << *index << '\n';
    }
}
