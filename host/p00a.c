#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include "p00.h"

uint8_t i;

int main() {
    setLanguage(de);
    const char* const s = tr(text2);
    printf("%s\n", s);
}
