#pragma once

#ifdef STM32G431xx
# include <stm32g4xx.h>
  namespace Mcu::Stm {
      struct Stm32G431;
  }
#else
# error "no MCU defined" 
#endif

using DefaultMcu = Mcu::Stm::Stm32G431;


