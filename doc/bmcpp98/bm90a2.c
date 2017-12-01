#include <stdint.h>

typedef struct T {
    uint8_t val;
    const __flash struct T *leaf[2];
} T;

const __flash T a, b, c;

const __flash T a = { 'a', { &c, &b } };
const __flash T b = { 'b', { &a, &b } };
const __flash T c = { 'c', { &b, &a } };

int get2(int id)
{
    return a.leaf[1]->leaf[0]->leaf[id]->leaf[1]->leaf[0]->val;
}

volatile uint8_t id = 1;

int main() {
    return get2(id); // make sure runtime-code is generated
}