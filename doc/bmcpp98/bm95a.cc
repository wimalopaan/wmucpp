#define NDEBUG

#include <cassert>
#include <cstdint>
#include <tuple>
#include "util/tuple.h"

template<typename... CC>
struct Node {
    Node(uint8_t v, CC... cc) : mValue{v}, mChildren{cc...} {}
    uint8_t value() const {return mValue;}
    void value(uint8_t v) {mValue = v;}
    uint8_t mValue{};
    std::tuple<CC...> mChildren;
};

namespace {
    auto a = Node{1};
    auto b = Node{2};
 
    auto m = Node{3, &a, &b};
    
    auto c = Node{4};

    auto top = Node{3, &c, &m};
    
}

volatile uint8_t value = 42;

template<typename Node>
void traverse(Node* node, const auto& callable) {
    if (!node) return;
    Meta::visit(node->mChildren, callable);
    callable(node);
}
int main() {
    uint8_t sum = 0;
    traverse(&top, [&](auto* node) {
        assert(node);
        sum += node->value();
        node->value(value); // make sure runtime-code is generated
    });
    return sum;
}


