#pragma once

namespace Mcu::Stm {
    struct Stm32G431;
    struct Stm32G473;
    struct Stm32G030;
    struct Stm32G031;
    struct Stm32G051;
    struct Stm32G0B1;
}
#if defined(STM32G431xx)
# include <stm32g4xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G431;
#elif defined(STM32G473xx)
# include <stm32g4xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G473;
#elif defined(STM32G030xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G030;
#elif defined(STM32G031xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G031;
#elif defined(STM32G051xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G051;
#elif defined(STM32G0B1xx)
# include <stm32g0xx.h>
  using DefaultMcu = Mcu::Stm::Stm32G0B1;
#else
# error "no MCU defined" 
#endif



