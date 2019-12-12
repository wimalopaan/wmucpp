#include <iostream>
#include <string>
#include <algorithm>
#include <iterator>
#include <array>
#include <type_traits>
#include <etl/meta.h>
//#define USE_FAKE_STDIN

namespace {
    template<typename T>
    auto contains(const T& container, typename T::value_type e) {
        return std::find(std::cbegin(container), std::cend(container), e) != std::cend(container);
    }
}

#include <iostream>
#include <string>

using namespace std;
const string punct {".,;?!_-'"};

bool is_punct(const char ch) {
    for(const char w : punct) {
        if(ch == w) return true;
    }
    return false;
}

void convert_punct(string& s) {
    bool convert{true};
    for(auto& ch : s) {
        if(ch == '"') {
            convert = !convert;
        }
        else { 
            if(convert && is_punct(ch)) ch = ' ';
        }
    }
}

//int main() {
//    cout << "Beliebige Eingabe machen\n";
//    cout << "Zum beenden Strg+Z eingeben\n";
    
//    for(string line; getline(cin, line);) {
//        convert_punct(line);
//        cout << line << '\n';
//    }
//}

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

int main() {
    constexpr auto exitChar{'~'};
    constexpr auto replace = Replace<'.', ',', ';', '-', '_', '!', '?'>::with<' '>();
    
    while(const auto inputChar{cin.get()}) {
        const auto replaceChar{replace(inputChar)};
        if (replaceChar == '~') {
            cout << exitChar << '\n' << "Programm wurde mit " << exitChar << " beendet." << '\n';             
            return 0;
        }
        cout << replaceChar;
    }   
}

//int main() {
//    constexpr std::array punctuation{'.', ',', ';', '-', '_', '!', '?'};
    
//#ifdef USE_FAKE_STDIN
//    std::string in{"hello world ohne . und , te~st"};
//#else
//    std::string in;
//    std::getline(std::cin, in);
//#endif
    
//    const auto end = std::find(std::begin(in), std::end(in), '~');
//    std::replace_if(std::begin(in), end, [&](const char c){ 
//        return contains(punctuation, c);
//    }, ' ');
    
//    std::copy(std::begin(in), end, std::ostream_iterator<char>(std::cout));
//}
