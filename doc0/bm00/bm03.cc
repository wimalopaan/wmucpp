#include <avr/io.h>

int  global_i;

extern const char* const __data_start;
extern const char* const __flash _etext;

int main(void)
{
    int i_from_flash   = *((const __flash int*)(_etext + ((const char*)&global_i   - __data_start)));
//  int i_from_flash   = *((_etext + ((const char*)&global_i   - __data_start)));
  return i_from_flash;
}
