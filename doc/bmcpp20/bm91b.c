#include <stdint.h>

volatile uint8_t result;
volatile uint8_t value = 100;

static const struct output_mv_strc {
    uint8_t fence;
    uint8_t out_o;
} output_mv_table[] = {
    {160,  4},
    { 80,  3},
    { 40,  2},
    { 20,  1},
    { 10,  0},
    { 0 }
};

void set_output_mv(const struct output_mv_strc* table, uint8_t output_mv) {
    for (uint8_t i = 0; table[i].fence; ++i) {
        if (output_mv < table[i].fence) {
            result = table[i].out_o;
        }
    }
}

int main() {
    set_output_mv(output_mv_table, value);
}
