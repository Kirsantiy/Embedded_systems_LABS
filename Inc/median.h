/*
 * median.h
 *
 *  Created on: May 8, 2023
 *      Author: pablo
 */

#ifndef INC_MEDIAN_H_
#define INC_MEDIAN_H_

#include "main.h"
#define MEDIAN_BUFFER_SIZE 8

typedef struct {
    int16_t data[MEDIAN_BUFFER_SIZE];
    int16_t sorted[MEDIAN_BUFFER_SIZE];
    uint8_t index;
} median_filter_t;

void median_filter_init(median_filter_t* filter);

int16_t median_filter_update(median_filter_t* filter, int16_t value);

#endif /* INC_MEDIAN_H_ */
