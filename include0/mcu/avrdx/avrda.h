#pragma once

#include <array>
#include <type_traits>
#include <etl/algorithm.h>

#if defined(__AVR_AVR128DA32__)
# include "avr128da32.h"
#elif defined(__AVR_AVR128DA48__)
# include "avr128da48.h"
#endif
