#include <algorithm>
#include <array>
#include <experimental/array>
#include <utility>
#include <optional>
//#include <util/algorithm.h>
#include <iostream>

struct RightOpen;
struct LeftOpen;
struct Closed;
struct Open;

struct Adjacent;
struct Disjoint;

template<typename T, typename Type>
struct Interval {
    typedef T value_type;
    typedef Type kind_type;
    constexpr bool includes(T v) const {
        if constexpr(std::is_same_v<Type, RightOpen>) {
            return (v >= left) && (v < right);
        }
        else {
            // ...
        }
    }
    T left{};
    T right{};
};

template<typename T, typename Type>
struct Boundary {
    typedef T value_type;
    constexpr bool includes(T v) const {
        if constexpr(std::is_same_v<Type, RightOpen>) {
            return (v < upper);
        }
        else {
            // ...
        }
    }
    T upper{};
};

template<typename Type, typename Construct, typename Limits>
constexpr auto make_intervals(const Limits& limits) {
    if constexpr(!std::is_same_v<Construct, Adjacent>) {
        using item_type = typename Limits::value_type;
        std::array<Interval<item_type, Type>, limits.size() - 1> in;
        for(typename Limits::size_type n = 0; n < in.size(); ++n) {
            in[n].left = limits[n];
            in[n].right = limits[n + 1];
        }
        return in;
    }
    else {
        auto sorted = limits;
//        Util::sort(sorted);        
        using item_type = typename Limits::value_type;
        std::array<Boundary<item_type, Type>, limits.size()> in;
        for(typename Limits::size_type n = 0; n < in.size(); ++n) {
            in[n].upper = limits[n];
        }
        return in;
    }
}

template<typename T, typename Intervals, typename Function>
constexpr void lookup(const T& value, const Intervals& intervals, const Function& f) {
    for(typename Intervals::size_type n = 0; n < intervals.size(); ++n) {
        if (intervals[n].includes(value)) {
            f(n);
            break;
        }
    }
}

volatile int result = -1;
volatile int value = 5;

constexpr auto limits = std::experimental::make_array(0, 10, 20, 40, 80, 160);
//constexpr auto intervals = make_intervals<RightOpen, Adjacent>(limits);
constexpr auto intervals = make_intervals<RightOpen, Disjoint>(limits);

int main() {
    lookup(value, intervals, [](auto index){result = index;});
    
    std::cout << "Result: " << result << '\n';
}
