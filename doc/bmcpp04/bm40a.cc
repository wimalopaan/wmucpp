#include "util/types.h"
#include "util/bits.h"
#include "util/meta.h"
#include "simavr/simavrdebugconsole.h"
#include "console.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

template<auto... Sizes>
struct BitField {
    inline static constexpr auto size = (Sizes + ...);
    inline static constexpr auto member_number = sizeof...(Sizes);
    typedef typename Util::TypeForBits<size>::type value_type;
    typedef Meta::List<bitsN_t<Sizes>...> field_types;
    typedef Meta::List<std::integral_constant<size_t, Sizes>...> size_types;
    typedef Meta::partial_sum<Meta::concat<Meta::List<std::integral_constant<size_t, 0>>, size_types>> begin_types;

    template<auto N>
    struct lowestBit {
        inline static constexpr size_t value = Meta::nth_element<N, begin_types>::value;
    };
    template<auto N>
    struct highestBit {
        inline static constexpr size_t value = Meta::nth_element<N, begin_types>::value + Meta::nth_element<N, size_types>::value - 1;
    };
    template<auto N>
    struct mask {
        inline static constexpr value_type value = ((1 << (highestBit<N>::value + 1)) - 1) - ((1 << lowestBit<N>::value) - 1);
    };
    
    template<auto N>
    Meta::nth_element<N, field_types> get() volatile {
        static_assert(N < member_number);
        return BitRange<value_type, highestBit<N>::value, lowestBit<N>::value>::convert(mData);
    }
    template<auto N>
    void increment() volatile {
        static_assert(N < member_number);
        value_type v = mData;
        v += (1 << lowestBit<N>::value);
        mData = (mData & ~mask<N>::value) | (v & mask<N>::value);
    }
    
    value_type mData{};
};


volatile BitField<2, 2, 2> bf;

int main() {
    bf.increment<1>();
    bf.increment<1>();
    bf.increment<0>();
//    bf.increment<1>();
}
