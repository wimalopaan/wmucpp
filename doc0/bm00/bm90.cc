#include <mcu/avr.h>

#include <utility>
#include <array>
#include <variant>
#include <type_traits>

#if 0 
namespace std {
    namespace detail {
        template<typename L>
        struct all_pointer {
            inline static constexpr bool value = (Meta::size_v<Meta::filter<std::is_pointer, L>> == Meta::size_v<L>);
        };
        template<typename L>
        inline constexpr bool all_pointer_v = all_pointer<L>::value;
    }
    template<typename... Tt>
    requires (detail::all_pointer_v<Meta::List<Tt...>>)
    struct variant<Tt...> {
        static_assert(Meta::is_set_v<Meta::List<Tt...>>, "types must be all different");
        inline static constexpr auto ramstart = std::integral_constant<uintptr_t, RAMSTART>::value;
        inline static constexpr auto ramend   = std::integral_constant<uintptr_t, RAMEND>::value;
        inline static constexpr auto ramsize  = std::integral_constant<uintptr_t, RAMSIZE>::value;
        
        inline static constexpr auto pointerBits = etl::minimumBitsForValue(ramsize - 1);
        inline static constexpr uintptr_t ptr_mask = (1 << pointerBits) - 1;
        //        std::integral_constant<uintptr_t, pointerBits>::_;
        
        inline static constexpr auto pointerShift = (sizeof(uintptr_t) * 8) - pointerBits; 
        inline static constexpr uintptr_t indexMask= (1 << pointerShift) - 1;
        
        template<typename U>
        requires (Meta::contains_v<Meta::List<Tt...>, U>)
                inline constexpr /*explicit */ variant(U&& v) : mValue{[&]{
//            uintptr_t val = reinterpret_cast<uintptr_t>(v - RAMSTART);
            uintptr_t val = reinterpret_cast<uintptr_t>(v);
            constexpr uintptr_t index = Meta::index_v<Meta::List<Tt...>, U>;
            return val | (index << pointerBits);
//            return (val << pointerShift) | index;
        }()}{}
        
//        template<typename F>
//        inline constexpr void visit(F&& f)  {
//        }
        
        template<typename F>
        inline constexpr void visit(F&& f) const {
            uint8_t i = (mValue >> pointerBits);
//            uint8_t i = (mValue & indexMask);
            Meta::visitAt<Meta::List<Tt...>>(i, [&](auto w){
                using ptr_type = decltype(w)::type;
                const ptr_type p = reinterpret_cast<ptr_type>(mValue & ptr_mask);
//                const ptr_type p = reinterpret_cast<ptr_type>(mValue >> pointerShift);
                f(p);
            });
        }
        
        private:
        uintptr_t mValue;
    };
}
#endif

struct A {
    inline constexpr uint8_t get() {
        return value;
    }
    inline void set(uint8_t v) {
        PORTA.OUT = v;
    }
    inline void toggle(uint8_t v) {
        PORTA.OUTTGL = v;
    }
    uint8_t value;
};
struct B {
    inline constexpr uint16_t get() {
        return value;
    }
    inline void set(uint8_t v) {
        PORTB.OUT = v;
    }
    inline void toggle(uint8_t v) {
        PORTB.OUTTGL = v;
    }
    uint16_t value;
};

namespace  {
    A x1{32};
    B x2{42};
    A x3{52};   
    //    float z;
}


int main() {
    std::array<std::variant<B*, A*>, 6> p{&x1, &x2, &x3, &x1, &x2, &x3};
    
//    p[0] = &x3;
    
    uint16_t x = 10;
    
//    p[0].visit([&](const auto& v){
//        x = v->get(); 
//    });
    
//    return x;
    
    //    for(uint8_t i = 0; i < p.size(); ++i) {
    //        p[i].visit([&](const auto& v){
    //            x += v->get();
    //        });
    //    }
    
    while(true) {
        for(const auto& i : p) {
            i.visit([&](const auto& v){
                x += v->get();
                v->set(x);
                v->toggle(0x01);
            });
        }
    }
    return x;
}
