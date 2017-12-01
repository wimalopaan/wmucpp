#define NDEBUG

#include <cassert>
#include <cstdint>
#include <array>

struct Node {
    uint8_t mValue{};
    std::array<Node*, 8> mChildren{}; // bad
};

namespace {
    Node a{1};
    Node b{2};
 
    Node m{3, &a, &b};
    
    Node c{4};
    Node top{5, &c, &m};
}

volatile uint8_t value = 42;

void traverse(Node* node, const auto& callable) {
    if (!node) return;
    for(auto& child: node->mChildren) {
        traverse(child, callable);
    }    
    callable(node);
}
int main() {
    uint8_t sum = 0;
    traverse(&top, [&](Node* node) {
        assert(node);
        sum += node->mValue;
        node->mValue = value; // make sure runtime-code is generated
    });
    return sum;
}


