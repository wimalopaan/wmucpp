#include <algorithm>
#include <array>
#include <utility>
#include <optional>
#include <util/algorithm.h>
#include <util/meta.h>
#include <util/bits.h>

struct RightOpen;
struct LeftOpen;
struct Closed;
struct Open;

struct Adjacent;
struct Disjoint;

template<typename Kind, typename Lower, typename Upper>
struct StaticInterval {
    typedef Kind kind_type;
    typedef Lower lower_type;
    typedef Upper upper_type;
    typedef typename Lower::value_type value_type;
    static_assert(std::is_same_v<typename upper_type::value_type, value_type>, "different value types not possible");
    static constexpr bool includes (value_type v) {
        if constexpr(std::is_same_v<kind_type, RightOpen>) {
            return (v >= lower_type::value) && (v < upper_type::value);
        }
        else {
            // ...
        }
    }
};

template<typename Kind, typename Upper>
struct StaticBoundary {
    typedef Kind kind_type;
    typedef Upper upper_type;
    typedef typename Upper::value_type value_type;
    static constexpr bool includes(value_type v) {
        if constexpr(std::is_same_v<kind_type, RightOpen>) {
            return (v < upper_type::value);
        }
        else {
            // ...
        }
    }
};

template<typename Kind, typename Arrangement, typename T, T... Values>
struct Lookup {
    static_assert([]{
        T values[] = {Values...};
        for(size_t i = 0; i < (sizeof...(Values) - 1); ++i) {
            if (values[i] > values[i + 1]) {
                return false;
            }
        }
        return true;
    }(), "use ascending order");
    static_assert(sizeof...(Values) > 0, "at least one value required");

    using value_list = Meta::List<std::integral_constant<T, Values>...>;
    using lower_list = Meta::pop_back<value_list>;
    using upper_list = Meta::pop_front<value_list>;
    
    template<typename Lower, typename Upper>
    using buildInterval = StaticInterval<Kind, Lower, Upper>;
    
    using intervals = Meta::transform2<buildInterval, lower_list, upper_list>;
    
    template<typename Boundary>
    using buildBoundary = StaticBoundary<Kind, Boundary>;
    
    using boundaries = Meta::transform<buildBoundary, value_list>;
    
    using thr = std::conditional_t<std::is_same_v<Arrangement, Disjoint>, intervals, 
                                   std::conditional_t<std::is_same_v<Arrangement, Adjacent>, boundaries, void>>;
    
    using numbered_intervals = Meta::make_numbered<thr>;

    template<typename... Nodes>
    struct Lutter {
        typedef Lutter type;
        template<typename Function>
        static constexpr void convert(T value, const Function& f) {
            ((Nodes::type::includes(value) ? (f(Nodes::index), false) : true) && ...);
        }        
    };
    
    using lut = Meta::apply<Lutter, numbered_intervals>;
        
    template<typename Function>
    static constexpr void convert(T value, const Function& f) {
        lut::convert(value, f);
    }
};

namespace Tests {
    static_assert([]{
        uint8_t result = -1;
        using lut = Lookup<RightOpen, Adjacent, uint8_t, 0, 10, 20, 40, 80, 160>;
        lut::convert(156, [&](auto index){result = index;});
        if (result != 5) return false;
        lut::convert(5, [&](auto index){result = index;});
        if (result != 1) return false;
        return true;
    }(), "test failed");    
}

volatile uint8_t result = -1;
volatile uint8_t value = 159;

int main() {
        using lut = Lookup<RightOpen, Adjacent, uint8_t, 0, 10, 20, 40, 80, 160>;
//    using lut = Lookup<RightOpen, Adjacent, uint8_t, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110>;
//    using lut = Lookup<RightOpen, Disjoint, uint8_t, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110>;
    lut::convert(value, [](auto index){result = index;});
}

