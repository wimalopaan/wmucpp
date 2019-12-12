#include <algorithm>
#include <array>
#include <type_traits>
#include <etl/meta.h>


template<auto... CC>
class Replace final {
    static_assert(sizeof...(CC) > 0, "at least one replacement item must be given");
    using replace_char_list = Meta::List<std::integral_constant<decltype(CC), CC> ...>;
    using value_type = Meta::front<replace_char_list>::value_type;
    static_assert(Meta::all_same_v<value_type, Meta::List<decltype(CC)...>>, "all replaced items must be of same type");
    static_assert(Meta::is_set_v<replace_char_list>, "replaced items must form a set");
    
    template<value_type W> 
    struct Replace_impl {
        constexpr value_type operator()(const value_type& in) const {
            if (((in == CC) || ...)) {
                return W;
            }
            return in;
        }
    };
public:
    Replace() = delete;
    template<value_type W>
    inline static constexpr auto with() {
        static_assert(!Meta::contains_v<replace_char_list, std::integral_constant<value_type, W>>, "replace item must be different the replaced items");
        return Replace_impl<W>{};
    }
};

struct Uart {
    inline static volatile std::byte txd;    
    inline static volatile std::byte rxd;    
};

int main() {
    constexpr auto exitChar{'~'};
    constexpr auto replace = Replace<'.', ',', ';', '-', '_', '!', '?'>::with<' '>();
    
    while(const char inputChar = (char)Uart::rxd) {
        const auto replaceChar{replace(inputChar)};
        if (replaceChar == exitChar) {
            Uart::txd = std::byte{'@'};
            return 0;
        }
        Uart::txd = std::byte{replaceChar};
    }   
}

