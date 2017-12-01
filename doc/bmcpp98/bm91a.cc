#define NDEBUG

#include <cassert>
#include <cstdint>
#include <array>

#ifndef NDEBUG
#include "console.h"
#include "simavr/simavrdebugconsole.h"
using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;
#endif

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
#ifndef NDEBUG
    std::outl<terminal>(sum);
#endif
    return sum;
}

#ifndef NDEBUG
void assertFunction(const PgmStringView& expr, const PgmStringView& file, unsigned int line) noexcept {
    std::outl<terminal>("Assertion failed: "_pgm, expr, Char{','}, file, Char{','}, line);
    while(true) {}
}
#endif

