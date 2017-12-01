#include <stdint.h>

typedef struct T {
    uint8_t val;
    struct T *leaf[2];
} T;

T a, b, c;

T a = { 'a', { &c, &b } };
T b = { 'b', { &a, &b } };
T c = { 'c', { &b, &a } };

int get2(int id)
{
    return a.leaf[1]->leaf[0]->leaf[id]->leaf[1]->leaf[0]->val;
}

volatile uint8_t id = 1;

int main() {
    return get2(id); // make sure runtime-code is generated
}
