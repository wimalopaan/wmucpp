#include <stdint.h>

typedef uint32_t tt;

tt f (tt);

tt call (tt x) {
    return f (x);
}

int main() {
}
