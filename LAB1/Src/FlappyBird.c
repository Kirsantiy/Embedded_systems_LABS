/*
 * FlappyBird.c
 *
 *  Created on: 29 нояб. 2023 г.
 *      Author: Kirs
 */

#include "FlappyBird.h"

void FlappyBird_init(FlappyBird_t *buffer) {
	memset(buffer->data, 0, sizeof(buffer->data));
	buffer->index = 0;
}

int16_t FlappyBird_update(FlappyBird_t *buffer, int16_t adc_value) {
	int16_t max_value = buffer->data[0];
	int8_t y0_bird_definition;
	uint8_t i;

	// Add the new value to the circular buffer
	buffer->data[buffer->index] = adc_value;

	for (uint8_t i = 1; i < BUFFER_SIZE; i++) {
		if (buffer->data[i] > max_value) {
			max_value = buffer->data[i];
		}
	}

	// Update the circular buffer index
	buffer->index = (buffer->index + 1) % BUFFER_SIZE;


	if (max_value < 300 && max_value > -300) {
			y0_bird_definition = 14;
		} else if ((max_value > 300 && max_value < 420)
				|| (max_value < -300 && max_value > -420)) {
			y0_bird_definition = 13;
		} else if ((max_value > 300 && max_value < 420)
				|| (max_value < -300 && max_value > -420)) {
			y0_bird_definition = 12;}
		else if ((max_value > 420 && max_value < 540)
				|| (max_value < -420 && max_value > -540)) {
			y0_bird_definition = 11;
		} else if ((max_value > 540 && max_value < 660)
				|| (max_value < -540 && max_value > -660)) {
			y0_bird_definition = 10;
		} else if ((max_value > 660 && max_value < 780)
				|| (max_value < -660 && max_value > -780)) {
			y0_bird_definition = 9;
		} else if ((max_value > 780 && max_value < 900)
				|| (max_value < -780 && max_value > -900)) {
			y0_bird_definition = 8;
		} else if ((max_value > 900 && max_value < 1020)
				|| (max_value < -900 && max_value > -1020)) {
			y0_bird_definition = 7;
		} else if ((max_value > 1020 && max_value < 1140)
				|| (max_value < -1020 && max_value > -1140)) {
			y0_bird_definition = 6;
		} else if ((max_value > 1140 && max_value < 1260)
				|| (max_value < -1140 && max_value > -1260)) {
			y0_bird_definition = 5;
		} else if ((max_value > 1260 && max_value < 1380)
				|| (max_value < -1260 && max_value > -1380)) {
			y0_bird_definition = 4;
		} else if ((max_value > 1380 && max_value < 1500)
				|| (max_value < -1380 && max_value > -1500)) {
			y0_bird_definition = 3;
		} else if ((max_value > 1500) || (max_value < -1500)) {
			y0_bird_definition = 2;
		}

	return y0_bird_definition;
}
