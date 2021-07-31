#include <utility>

struct MCU {
    struct P1 {
        static inline volatile std::byte r1;
        static inline volatile std::byte r2;
    };
    struct P2 {
        static inline volatile std::byte r1;
        static inline volatile std::byte r2;
    };
};

template<auto N, typename P>
struct Per {
    static void init(const std::byte v){
        value = v;
    }
    static std::byte get(){
        return value;
    }
    static void set(const std::byte v) {
        P::r1 = v;
    }
    static void toggle(const std::byte v) {
        P::r1 |= v;
    }
    static void store() {
        value = P::r2;
    }
    static inline std::byte value{};
};

template<typename... TT> struct List {};

template<typename> struct Visit;
template<typename... II>
struct Visit<List<II...>> {
    static void all(const auto& c) {
        (c(II{}),...);
    }
};

namespace detail {
    template<typename L> struct front_impl;
    
    template<typename F, typename... II>
    struct front_impl<List<F, II...>> {
        using type = F;
    };
    
    template<typename L> struct back_impl;
    template<typename... II>
    struct back_impl<List<II...>> {
        using type = decltype((II{}, ...));
    };
}

template<typename L>
using front = detail::front_impl<L>::type;

template<typename L>
using back= detail::back_impl<L>::type;

int main() {
    using a = Per<0, MCU::P1>;
    using b = Per<0, MCU::P2>;
    using c = Per<1, MCU::P1>;
    using d = Per<1, MCU::P2>;
    
    using list = List<a, b, c, d>;

    a::init(32_B);
    b::init(42_B);
    c::init(52_B);
    d::init(62_B);
    
    while(true) {
        std::byte x{};
        
        Visit<list>::all([&]<typename C>(C){
                             C::store();
                             x |= C::get();
                    });
        
        front<list>::set(x);
    }
}

