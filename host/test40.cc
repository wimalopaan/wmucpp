#include <cstdint>
#include <array>
#include <string>
#include <iostream>

struct MenuItem {
    MenuItem() = default;
    explicit MenuItem(const std::string& t) : mTitle(t) {}
    virtual size_t numberOfChilden() const {return 0;}
    virtual MenuItem* child(size_t) const {return nullptr;}
   std::string mTitle;
};

template<auto N>
class Menu : public MenuItem {
public:
    template<typename... CC>
    Menu(CC... cc) : mChildren{cc...} {}
    virtual size_t numberOfChilden() const override {return N;}
    virtual MenuItem* child(size_t n) const override {return mChildren[n];}
private:
    std::array<MenuItem*, N> mChildren;
};
template<typename... CC>
Menu(CC... cc) -> Menu<sizeof...(CC)>;


auto sm1 = MenuItem{"a"};
auto sm2 = MenuItem{"b"};
auto m1 = Menu{&sm1, &sm2};

auto sm3 = MenuItem{"c"};
auto m2 = Menu{&sm3};

auto m3 = Menu{&m1, &m2};

void print(const MenuItem* m) {
}

MenuItem* select(MenuItem* mi, size_t n) {
    if (n < mi->numberOfChilden()) {
        return mi->child(n);
    }
    return nullptr;
}

int main() {
    MenuItem* mi = &m3;
    while(true) {
        print(mi);
        size_t choice = 0;
        std::cin >> choice;
        mi = select(mi, choice);
    }
}
