#include <stdint.h>
#include <stdbool.h>

void bad1();
void bad2();
void bad3();

bool FOO_EQ_U32( uint32_t a, uint32_t b) {
   return ( a == b );
}

bool FOO_EQ_S32( int32_t a, int32_t b) {
   return ( a == b );
}

#define FOO_EQ_NOK(a, b)   _Generic((a), \
                            uint32_t:  _Generic((b), \
                                           uint32_t:   FOO_EQ_U32 \
                                       ), \
                            int32_t:   _Generic((b), \
                                           int32_t:   FOO_EQ_S32 \
                                        ) \
                           ) ((a),(b))

#define FOO_EQ_OK(a, b)   _Generic((a), \
                            uint32_t:  _Generic((b), \
                                           uint32_t:  FOO_EQ_U32, \
                                           default: bad1 \
                                       ), \
                            int32_t:   _Generic((b), \
                                           int32_t:   FOO_EQ_S32, \
                                           default: bad2 \
                                        ), \
                            default: bad3 \
                           ) ((a),(b))


int main() {
//    bool isEQ = FOO_EQ_NOK((uint32_t)1, (uint32_t)(2));

    uint32_t x = 0;
    int32_t y = 0;
    
//    bool isEQ = FOO_EQ_NOK(x, x);
    
    bool isEQ = FOO_EQ_OK(x, x);
    
}