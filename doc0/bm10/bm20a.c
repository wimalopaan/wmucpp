#include <stdint.h>
#include <stddef.h>

#include "bm20.h"

uint8_t i;

int main() {
    setLanguage(de);
    const char* const s = tr(text1);
    
    return s[i];
}
