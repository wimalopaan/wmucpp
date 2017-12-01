#include <cstdint>

struct T {
    uint8_t val;
    struct T* leaf[2]{};
};

T a{1};
T b{2};
T c{3};

int get2(uint8_t id)
{
    return a.leaf[1]->leaf[0]->leaf[id]->leaf[1]->leaf[0]->val;
}

volatile uint8_t id;

int main() {
    a.leaf[0] = &c;
    a.leaf[1] = &b;
    b.leaf[0] = &a;
    b.leaf[1] = &b;
    c.leaf[0] = &b;
    c.leaf[1] = &a;
    return get2(id); // make sure runtime-code is generated
}