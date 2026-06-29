#include "stm32g0xx.h"

#define VECT_TAB_OFFSET  0x0U /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x100. */
void SystemInit(){
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
}
