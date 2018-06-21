#include <stdint.h>
#include <stdio.h>

#define TFT_FKT(x)      tft_wr( sizeof(x) - 1, x, 1 )

uint8_t tft_wr( uint8_t len, const uint8_t *dat, bool flash ) {
    printf("len=%d\n", len);
    return len;
} 
uint8_t INIT[] =
                "\x1bTC\0"              // text cursor off
                "\x1b""DO\2"            // rotate 180°
                "\x1b""FD\x8\x1"        // display color
                "\x1b""FZ\x8\x1"        // text color
                "\x1b""DL"              // display clear
                "\x1b""YZ\x0"           // no delay
                "\x1b""YH\x64"          // light on
                "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz"
                "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz"
                "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz"
                ;

void init(const uint8_t* s) {
    TFT_FKT(s);
}

int main() {
    TFT_FKT(INIT);
    init(INIT);
}
