#pragma once

#define PGMSUFFIX pgm
#define PASTER(x,y) x ## _ ## y
#define EVALUATOR(x,y)  PASTER(x,y)
#define ASSERTSTRING(string) EVALUATOR(string, PGMSUFFIX)

#include "../pgm/pgmstring.h"


