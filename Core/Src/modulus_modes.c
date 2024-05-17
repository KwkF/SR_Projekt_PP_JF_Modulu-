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

	arm_rfft_fast_init_f32 (&fft, 2048);

	float rfft[2048]={0};

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

		arm_rfft_fast_init_f32 (&fft, 2048);

		float rfft[2048]={0};

		arm_rfft_fast_f32(&fft, input, rfft, 0);
		// vector

		float out_fft[2048]={0};

		memmove(out_fft,rfft + 10*sizeof(float),(2048-10)*sizeof(float));

		for(size_t i=0;i<2048-100;++i)
		{
			out_fft[i+100]=rfft[i];
		}

		// rfft
		arm_rfft_fast_f32(&fft, out_fft, output, 1);

		// output
}

void low_freq_dummping(float* input,float* output)
{
	// fft

		arm_rfft_fast_instance_f32 fft;

		arm_rfft_fast_init_f32 (&fft, 2048);

		float rfft[2048]={0};

		arm_rfft_fast_f32(&fft, input, rfft, 0);
		// vector

		for(uint16_t i=1;i<1024;++i)
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

		arm_rfft_fast_init_f32 (&fft, 2048);

		float rfft[2048]={0};

		arm_rfft_fast_f32(&fft, input, rfft, 0);
		// vector

		for(uint16_t i=1024;i<2048;++i)
		{
			rfft[i]=rfft[i]*0.5f;
		}

		// rfft
		arm_rfft_fast_f32(&fft, rfft, output, 1);

		// output
}

modulus_mode modes_list[MODES_COUNT]={&dummy_mode,&freq_shift,&low_freq_dummping,&high_freq_dummping};

