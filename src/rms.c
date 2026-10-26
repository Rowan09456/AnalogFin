#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

float calculate_rms(const uint16_t *signal, uint16_t length, float conversion_factor)
{
    // Edge case: if array is empty, RMS is 0
    if (length == 0) {
        return 0.0;
    }

    float sum_squares = 0.0;

    for (uint16_t i = 0; i < length; i++) {
        sum_squares += signal[i] * signal[i] * conversion_factor * conversion_factor; // Convert to 3.3v and square
    }

    float mean_square = sum_squares / (float)length;

    return sqrt(mean_square);
}