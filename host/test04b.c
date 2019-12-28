#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

static_assert(true);

//typedef char static_assertion_bufer_size_wrong[(sizeof(int)==8)?1:-1];

int main() {
    static_assert(true);
    printf("S: %ld\n", sizeof(int));
    printf("A\n");
    if (1 / ((sizeof(int) == 0) ? 1 : -1)) {}
    printf("A\n");

//    typedef char static_assertion_bufer_size_wrong[(sizeof(int)==8)?1:-1];
}
