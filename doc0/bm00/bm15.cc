#include <cassert>
#include <cstdint>
#include <type_traits>
#include <limits>
#include <algorithm>
#include <tuple>

#include <mcu/pgm/pgmstring.h>

#include <etl/tuple.h>
#include <etl/type_traits.h>

namespace etl::CodeData {
    namespace detail {
        template<typename...> struct List{};
        template<typename> struct visit;
        template<typename F, typename... T>
        struct visit<List<F, T...>> {
            template<typename C>
            static constexpr void at(uint8_t i, C f) {
                if (i == 0) {
                    f(F{});
                }
                else {
                    visit<List<T...>>::at(i - 1, f);
                }
            }            
        };
        template<typename T>
        struct visit<List<T>> {
            template<typename C>
            static constexpr void at(uint8_t i, C f) {
                if (i == 0) {
                    f(T{});
                }
                else {
                    assert(false);
                }
            }            
        };
    }
    template<typename C, C... CC> 
    struct String {
        using list = detail::List<std::integral_constant<C, CC>...>;
        constexpr C operator[](uint8_t index) const {
            C value{};
            detail::visit<list>::at(index, [&]<auto V>(std::integral_constant<C, V>){
                                        value = V;              
                                    });
            return value;
        }
        constexpr size_t size() const {
            return sizeof...(CC);
        }
    };
    
    template<typename> struct isString : std::false_type {};
    
    template<typename C, C... CC>
    struct isString<String<C, CC...>> : std::true_type {}; 
    
    template<typename C, C... CC>
    static constexpr bool isString_v = isString<C, CC...>::value;
}

template<typename C, C... CC>
consteval /*constexpr */etl::CodeData::String<C, CC...> operator"" _cds(){
    return etl::CodeData::String<C, CC...>{};
}


template<class... Ts> struct overload_t : Ts... { using Ts::operator()...; };
template<class... Ts> overload_t(Ts...) -> overload_t<Ts...>;

namespace {
    std::tuple t{uint8_t{1}, uint16_t{2}, "abc"_pgm};
}

int main() {
    std::byte bits{};
    
#if 0
    etl::visit(t, [&]<typename T>(T v){
                   if constexpr(std::is_same_v<T, uint8_t>) {
                       if (v == 1) {
                           bits |= std::byte{0x01};
                       }
                   }
                   else if constexpr(std::is_same_v<T, uint16_t>) {
                       if (v == 2) {
                           bits |= std::byte{0x02};
                       }
                   }
                   else if constexpr(AVR::Pgm::isString_v<T>) {
                       if constexpr (std::is_same_v<T, decltype("abc"_pgm)>) {
                           bits |= std::byte{0x04};
                       }
                   }
               });
#else
    etl::visit(t, overload_t{
                   [&](uint8_t v){
                       if (v == 1) {
                           bits |= std::byte{0x01};
                       }
                   },
                   [&](uint16_t v){
                       if (v == 2) {
                           bits |= std::byte{0x02};
                       }
                   },
                   [&]<typename S>(const S& v) requires (AVR::Pgm::isString_v<S>){
                       if constexpr (std::is_same_v<S, decltype("abc"_pgm)>) {
                           bits |= std::byte{0x04};
                       }
                   },
               });
#endif
    
//    auto s = "12345678901234567890"_cds;
    
//    for(uint8_t i = 0; i != s.size(); ++i) {
//        x = s[i];
//    }
//    return x;
    
    return int(bits);
}

