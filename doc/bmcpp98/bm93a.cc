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

template<auto N>
struct Node : Base {
    Node(uint8_t v) : mValue{v}{}
    uint8_t value() const override {return mValue;}
    void value(uint8_t v) override {mValue = v;}
    Base** begin() override {return mChildren.begin();}
    Base** end() override {return mChildren.end();}
    uint8_t mValue{};
    std::array<Base*, N> mChildren{};
    
};

namespace {
    Node<0> a{1};
    Node<0> b{2};
 
    auto m = []{
        Node<2> n{3};
        n.mChildren[0] = &a;
        n.mChildren[1] = &b;
        return n;
    }();
    
    Node<0> c{4};
    auto top = []{
        Node<2> n{5};
        n.mChildren[0] = &c;
        n.mChildren[1] = &m;
        return n;
    }();
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


