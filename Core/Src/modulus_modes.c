/*
 * modulus_modes.h
 *
 *  Created on: Apr 8, 2024
 *      Author: projectrobal
 */


#include "modulus_modes.h"

#include <stdint.h>

void dummy_mode(uint8_t* input,uint8_t* output);

modulus_mode modes_list[MODES_COUNT]={&dummy_mode};

