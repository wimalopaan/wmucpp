
#include "util/meta.h"
#include "util/algorithm.h"

struct A {
    typedef std::integral_constant<uint8_t, 0> v;
    inline static constexpr const char s[] = "abc";
};

struct B {
    typedef std::integral_constant<uint8_t, 2> v;
    inline static constexpr const char s[] = "def";
};

using l1 = Meta::List<A, B>;

volatile uint8_t x;
volatile uint8_t y = 2;

volatile const char xs[] = "abc";

int main() {
    x = Meta::find<l1>([](auto v){
        using type = typename decltype(v)::type;
        using value_type = type;
        return Util::compareFirstN<3>(value_type::s, xs);
    });
//    x = Meta::find<l1>([](auto v){
//        using type = typename decltype(v)::type;
//        using value_type = type;
//        return Util::compareElements<0, 1, 2>(value_type::s, xs);
//    });
    
//    x = Meta::find<l1>([](auto v){
//        using type = typename decltype(v)::type;
//        using value_type = typename type::v;
//        return ((value_type::value) == y);
//    });
//    x = Meta::find<l1>([](auto v){
//        using type = typename decltype(v)::type;
//        using value_type = typename type::v;
//        return ((value_type::value) == 2);
//    });
}

