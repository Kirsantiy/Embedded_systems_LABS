/*
 * FlappyBird.h
 *
 *  Created on: 29.11.2023
 *      Author: Kirs
 */

#ifndef INC_FLAPPYBIRD_H_
#define INC_FLAPPYBIRD_H_

#include "main.h"

#define BUFFER_SIZE 30

typedef struct {
    int16_t data[BUFFER_SIZE];
    uint8_t index;
} FlappyBird_t;

void FlappyBird_init(FlappyBird_t* buffer);

int16_t FlappyBird_update(FlappyBird_t* buffer, int16_t adc_value);

#endif /* INC_FLAPPYBIRD_H_ */
