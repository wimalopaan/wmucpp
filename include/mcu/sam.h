#pragma once

#include "mcu.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wexpansion-to-defined" // nicht möglich -> Makefile
#pragma GCC diagnostic ignored "-Wregister"

// in compiler.h
// #undef __always_inline

#include <samd21.h>

//#include <drivers/system/system.h>

#pragma GCC diagnostic pop
