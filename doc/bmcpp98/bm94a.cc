#define NDEBUG

#include <cassert>
#include <cstdint>
#include <array>

struct Base {
    virtual uint8_t value() const = 0;
    virtual void value(uint8_t v) = 0;
    virtual Base** begin() = 0;
    virtual Base** end() = 0;
};

template<typename... CC>
struct Node : Base {
    Node(uint8_t v, CC... cc) : mValue{v}, mChildren{cc...} {}
    uint8_t value() const override {return mValue;}
    void value(uint8_t v) override {mValue = v;}
    Base** begin() override {return mChildren.begin();}
    Base** end() override {return mChildren.end();}
    uint8_t mValue{};
    std::array<Base*, sizeof...(CC)> mChildren{};
    
};

namespace {
    auto a = Node{1};
    auto b = Node{2};
 
    auto m = Node{3, (Base*)&a, (Base*)&b};
    
    auto c = Node{4};

    auto top = Node{5, (Base*)&c, (Base*)&m};
    
}

volatile uint8_t value = 42;

void traverse(Base* node, const auto& callable) {
    if (!node) return;
    for(auto& child: *node) {
        traverse(child, callable);
    }    
    callable(node);
}
int main() {
    uint8_t sum = 0;
    traverse(&top, [&](Base* node) {
        assert(node);
        sum += node->value();
        node->value(value); // make sure runtime-code is generated
    });
    return sum;
}


