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

	arm_rfft_fast_instance_f32 fft;

	arm_rfft_fast_init_f32 (&fft, 4096);

	float rfft[4096]={0};

	arm_rfft_fast_f32(&fft, input, rfft, 0);
	// vector

	// do something

	// rfft
	arm_rfft_fast_f32(&fft, rfft, output, 1);

	// output
}

modulus_mode modes_list[MODES_COUNT]={&dummy_mode};

