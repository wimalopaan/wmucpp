#include <array>
#include "container/pgmarray.h"
#include "util/meta.h"
#include "util/concepts.h"

template<typename Generator>
class Converter {
    inline static constexpr auto mData = Generator()();    
    typedef typename decltype(mData)::size_type size_type;
    using index_list = std::make_integer_sequence<size_type, mData.size>;
    template<typename> struct Transform;
    template<auto... I, typename ValueType> 
    struct Transform<std::integer_sequence<ValueType, I...>> {
        typedef Meta::List<std::integral_constant<ValueType, mData[I]> ...> type;
    };
    template<typename> struct Pgm;
    template<auto... I, typename ValueType> 
    struct Pgm<std::integer_sequence<ValueType, I...>> {
        typedef Util::PgmArray<ValueType, mData[I] ...> type;
    };
    
    template<typename L, Util::Callable<size_type> C>
    static inline void at_impl(size_type index, const C& f) {
        if constexpr(Meta::size<L>::value > size_type{0}) {
            if (index == size_type{0}) {
                using first = Meta::front<L>;
                f(first::value);
            }
            else {
                at_impl<Meta::rest<L>>(size_type(index - 1), f);
            }
        }
        else {
            (void)index;
        }
    }
public:
    using array_type = typename Transform<index_list>::type;
    using pgm_type = typename Pgm<index_list>::type;
    
    template<Util::Callable<size_type> C>
    static inline void at(size_type index, const C& f) {
        at_impl<array_type>(index, f);
    }
};

volatile uint8_t y = 2;
volatile uint8_t x = 0;

int main() {
    struct A {
        constexpr auto operator()() {
            std::array<uint8_t, 16> a;
            for(uint8_t i = 0; i < a.size; ++i) {
                a[i] = 2 * i + 1;
            }
            return a;
        }
    };

    // besser: PgmArray!!!
    
    using c = Converter<A>;
//    c::at(y, [](uint8_t v){x = v;});    
    
    constexpr auto a = c::pgm_type{};
    
    x = a[y];
}
