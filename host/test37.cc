#include <algorithm>
#include <array>
#include <experimental/array>
#include <utility>
#include <optional>
#include <util/algorithm.h>
#include <util/meta.h>
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
        Util::sort(sorted);        
        using item_type = typename Limits::value_type;
        std::array<Boundary<item_type, Type>, limits.size()> in;
        for(typename Limits::size_type n = 0; n < in.size(); ++n) {
            in[n].upper = limits[n];
        }
        return in;
    }
}

template<typename Kind, typename Lower, typename Upper>
struct StaticInterval {
    typedef Kind kind_type;
    typedef Lower lower_type;
    typedef Upper upper_type;
    typedef typename Lower::value_type value_type;
    static_assert(std::is_same_v<typename upper_type::value_type, value_type>);
    static constexpr bool includes (value_type v) {
        if constexpr(std::is_same_v<kind_type, RightOpen>) {
            return (v >= lower_type::value) && (v < upper_type::value);
        }
        else {
            // ...
        }
    }
};

namespace detail {
    template<typename, typename> struct LUTConverter;
    template<typename... II, auto... NN>
    struct LUTConverter<Meta::List<II...>, std::index_sequence<NN...>> {
        typedef typename Meta::front<Meta::List<II...>>::value_type value_type;
    
        template<typename F>
        static constexpr void convert(value_type value, const F& f) {
            ((II::includes(value) ? (f(value_type(NN)), false) : true) && ...);
        }        
    };
}

template<typename Kind, typename Intervals, typename T, T... Values>
struct Lookup {
    using value_list = Meta::List<std::integral_constant<T, Values>...>;
    using lower_list = Meta::reverse<Meta::rest<Meta::reverse<value_list>>>;
    using upper_list = Meta::rest<value_list>;
    
    template<typename L, typename U>
    using buildInterval = StaticInterval<Kind, L, U>;
    
    using intervals = Meta::transform2<buildInterval, lower_list, upper_list>;

    using converter = detail::LUTConverter<intervals, std::make_index_sequence<Meta::size<intervals>::value>>;
    
    template<typename F>
    static constexpr void convert(T value, const F& f) {
        converter::convert(value, f);
    }
};


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
volatile int value = 159;

constexpr auto limits = std::experimental::make_array(0, 10, 20, 40, 80, 160);
//constexpr auto intervals = make_intervals<RightOpen, Adjacent>(limits);
constexpr auto intervals = make_intervals<RightOpen, Disjoint>(limits);

int main() {
//    lookup(value, intervals, [](auto index){result = index;});

    using lut = Lookup<RightOpen, Adjacent, uint8_t, 0, 10, 20, 40, 80, 160>;
    
    lut::convert(value, [](auto index){result = index;});
    
    std::cout << "Result: " << result << '\n';
}
