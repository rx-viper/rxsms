#ifndef BGTASK_SAMPLE_ADC_INPUTS_H
#define BGTASK_SAMPLE_ADC_INPUTS_H

#include <inttypes.h>
#include "task.h" /* for struct bgtask */

// TODO maybe set as const?
const struct bgtask bgtask_sample_adc_inputs;

struct
{
    uint16_t poti_bit_error_rate;
    uint16_t poti_blocking_rate;
    uint16_t poti_blocking_duration;
    uint16_t current_sense;
} adc_sense_buffer;

#endif /* BGTASK_SAMPLE_ADC_INPUTS_H */
