/*
 * modulus_modes.h
 *
 *  Created on: Apr 8, 2024
 *      Author: projectrobal
 */


#include "modulus_modes.h"

#include <stdint.h>

void dummy_mode(float* input,float* output)
{
	// fft
	// vector

	// do something

	// rfft

	// outpyt
}

modulus_mode modes_list[MODES_COUNT]={&dummy_mode};

