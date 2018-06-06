#include <algorithm>
#include <array>
#include <utility>
#include <optional>
#include <util/algorithm.h>
#include <util/meta.h>

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

namespace detail {
    template<typename, typename> struct LUTConverter;
    template<typename... II, auto... NN>
    struct LUTConverter<Meta::List<II...>, std::index_sequence<NN...>> {
        typedef Meta::List<typename II::value_type...> value_types;
        typedef typename Meta::front<value_types> value_type;
        static_assert(Meta::all_same<value_type, value_types>::value, "all intervals must use same value type");
        template<typename F>
        static constexpr void convert(value_type value, const F& f) {
            ((II::includes(value) ? (f(value_type(NN)), false) : true) && ...);
        }        
    };
}

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
    
    template<typename L, typename U>
    using buildInterval = StaticInterval<Kind, L, U>;
    
    using intervals = Meta::transform2<buildInterval, lower_list, upper_list>;
    using converterI = detail::LUTConverter<intervals, std::make_index_sequence<Meta::size<intervals>::value>>;
    
    template<typename U>
    using buildBoundary = StaticBoundary<Kind, U>;
    
    using boundaries = Meta::transform<buildBoundary, value_list>;
    
    using converterB = detail::LUTConverter<boundaries, std::make_index_sequence<Meta::size<boundaries>::value>>;
    
    template<typename F>
    static constexpr void convert(T value, const F& f) {
        if constexpr(std::is_same_v<Arrangement, Disjoint>) {
            converterI::convert(value, f);
        }
        else if constexpr(std::is_same_v<Arrangement, Adjacent>) {
            converterB::convert(value, f);
        }
        else {
            static_assert(Meta::always_false<T>::value);
        }
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
    using lut = Lookup<RightOpen, Adjacent, uint8_t, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110>;
    lut::convert(value, [](auto index){result = index;});
}
