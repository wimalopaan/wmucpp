#pragma once

#include <array>
#include <type_traits>
#include <etl/algorithm.h>

#include "../common/register.h"

#include <external/units/physical.h>

namespace AVR {
}

#if defined(__AVR_ATtiny412__)
# include "attiny412.h"
#endif
#if defined(__AVR_ATtiny1614__)
# include "attiny1614.h"
#endif
