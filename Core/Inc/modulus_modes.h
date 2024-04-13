/*
 * modulus_modes.h
 *
 *  Created on: Apr 8, 2024
 *      Author: projectrobal
 */

#ifndef INC_MODULUS_MODES_H_
#define INC_MODULUS_MODES_H_

#include <stdint.h>

// a first argument is input buffer and second is output buffer
// this is a pointer for a function that will handle the module modes

typedef void (*modulus_mode)(float*,float*);

#define MODES_COUNT 1


void dummy_mode(float* input,float* output);

extern modulus_mode modes_list[MODES_COUNT];


typedef struct modulus_config
{
	uint8_t mode;
} modulus_config_t;


#endif /* INC_MODULUS_MODES_H_ */
