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

	arm_rfft_fast_init_f32 (&fft, 512);

	float rfft[512]={0};

	arm_rfft_fast_f32(&fft, input, rfft, 0);
	// vector

	// do something

	// rfft
	arm_rfft_fast_f32(&fft, rfft, output, 1);

	// output
}

void freq_shift(float* input,float* output)
{
	// fft

		arm_rfft_fast_instance_f32 fft;

		arm_rfft_fast_init_f32 (&fft, 4096);

		float rfft[4096]={0};

		arm_rfft_fast_f32(&fft, input, rfft, 0);
		// vector

		float out_fft[4096]={0};

		memmove(out_fft,rfft + 10*sizeof(float),(4096-10)*sizeof(float));

		// rfft
		arm_rfft_fast_f32(&fft, rfft, output, 1);

		// output
}

void low_freq_dummping(float* input,float* output)
{
	// fft

		arm_rfft_fast_instance_f32 fft;

		arm_rfft_fast_init_f32 (&fft, 4096);

		float rfft[4096]={0};

		arm_rfft_fast_f32(&fft, input, rfft, 0);
		// vector

		for(uint16_t i=1;i<2048;++i)
		{
			rfft[i]=rfft[i]*0.5f;
		}

		// rfft
		arm_rfft_fast_f32(&fft, rfft, output, 1);

		// output
}

void high_freq_dummping(float* input,float* output)
{
	// fft

		arm_rfft_fast_instance_f32 fft;

		arm_rfft_fast_init_f32 (&fft, 4096);

		float rfft[4096]={0};

		arm_rfft_fast_f32(&fft, input, rfft, 0);
		// vector

		for(uint16_t i=2048;i<4096;++i)
		{
			rfft[i]=rfft[i]*0.5f;
		}

		// rfft
		arm_rfft_fast_f32(&fft, rfft, output, 1);

		// output
}

modulus_mode modes_list[MODES_COUNT]={&dummy_mode,&freq_shift,&low_freq_dummping,&high_freq_dummping};

