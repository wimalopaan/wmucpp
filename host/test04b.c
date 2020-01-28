#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>

ssize_t find_wrong(const char* const* array, const char* const pattern) {
    assert(array);
    const size_t length = sizeof(array) / sizeof(array[0]); // falsch
    for(size_t i = 0; i < length; ++i) {
        assert(array[i]);
        if (strcmp(pattern, array[i]) == 0) {
            return i;
        }
    }
    return -1;    
    
} 

ssize_t find(const char* const* it, const char* const pattern) {
    assert(it);
    for(size_t i = 0; it[i]; ++i) {
        assert(it[i]);
        if (strcmp(pattern, it[i]) == 0) {
            return i;
        }
    }
    return -1;    
    
} 

ssize_t commandindex(const char* const input) {
    assert(input);
	const char* const commands[] = {"","on","off","left","right","up","down","step","back","erase", NULL};
    return find(commands, input);
}

ssize_t commandindex2(const char* const input) {
    assert(input);
	const char* const commands[] = {"","on","off","left","right","up","down","step","back","erase"};
    return find_wrong(commands, input);
}

int main(const int argc, const char* const* argv) {
    assert(argc > 1);
    return commandindex2(argv[1]);
}

