#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void(*fsm_part_t)(void) ;

void circular_call(const fsm_part_t* const functions);
