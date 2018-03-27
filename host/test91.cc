#include <iostream>
#include <algorithm>
#include <array>


bool f(char x) {
    std::cout << __PRETTY_FUNCTION__ << x << '\n';
    return true;
}

bool doSth() {
    std::cout << __PRETTY_FUNCTION__ << '\n';
    return true;
}

template<typename... I, typename F>
void find_if_not(const F& f, const I... ii) {
    const auto c = {ii...};
    std::find_if_not(std::begin(c), std::end(c), f);
}

int main() {
    int x = 2;
    [&]{
        if(x == 1) {
            return;
        }
        std::cout << "D\n";
    }(); 

    
    char c = 'a';
    
    [](auto... cc) {
         (f(cc) && ... && doSth());
    }(c, '@', '$');
    
    for(auto& item : {c, '@', '$'}) {
        if (!f(item)) break;
    }
    
    find_if_not(f, c, '@', '$');
    
}
