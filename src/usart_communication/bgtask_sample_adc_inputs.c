#include <avr/io.h>
#include "bgtask_sample_adc_inputs.h"
#include "util.h"

/*
 * Ressources used _once_ by this task:
 * (none)
 *
 * Ressources _continously_ used by this task:
 * ADCA
 * ADCA.CH0..CH3
 * DMA
 * DMA.CH0
 * PORTA PIN 0..4
 */

/* "ADC_CH_NOGAIN_MUXNEG_PADGND_gc" missing in avr/io.h corresponds to 0x05 */
#define ADC_CH_NOGAIN_MUXNEG_PADGND_gc          0x05
/* "ADC_CH_WGAIN_MUXNEG_PADGND_gc" missing in avr/io.h corresponds to 0x07 */
#define ADC_CH_WGAIN_MUXNEG_PADGND_gc           0x07

#define ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc    ADC_CH_MUXPOS_PIN1_gc
#define ADC_CH_MUXPOS_POTI_BLOCKING_RATE_gc     ADC_CH_MUXPOS_PIN2_gc
#define ADC_CH_MUXPOS_POTI_BLOCKING_DURATION_gc ADC_CH_MUXPOS_PIN3_gc
#define ADC_CH_MUXPOS_CURRENT_SENSE_gc          ADC_CH_MUXPOS_PIN4_gc


static void init(void);
const struct bgtask bgtask_sample_adc_inputs = { .init = &init };

/// Reads the ADC calibration value and TEMPSENSE calibration value. Results are
/// stored in the public variables #adca_calibration and #tempsense_calibration.
static void
production_signature_row_read_calibration(uint16_t *adca_calibration, uint16_t *tempsense_calibration)
{
    uint8_t adcacal0_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_ADCACAL0;
    uint8_t adcacal1_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_ADCACAL1;
    uint8_t tempsense0_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_TEMPSENSE0;
    uint8_t tempsense1_addr = (uint8_t) (uint16_t) &PRODSIGNATURES_TEMPSENSE1;

    *adca_calibration = (prodsigrow_read_byte(adcacal1_addr) << 8)
        | prodsigrow_read_byte(adcacal0_addr);
    *tempsense_calibration = (prodsigrow_read_byte(tempsense1_addr) << 8)
        | prodsigrow_read_byte(tempsense0_addr);
}

/// Init the ADC to scan the potis and optional current sense input.
///
/// It uses the first ADC channel CH0 in FREERUNning mode and sets the
/// SCAN register to consecutively sample the inputs for the three potis
/// and the current sense input.
/// The results are written to the given buffer through DMA CH0.
static void
init(void)
{
    // TODO maybe enable TEMPREF and sample temperature once to seed the PRNG

    /* init ADCA and ADC CH0 to be used in FREERUN and SCAN mode */
    PORTA.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm;
    PORTA.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm;

    uint16_t adca_calibration, tempsense_calibration;
    production_signature_row_read_calibration(&adca_calibration,
           &tempsense_calibration);

    ADCA.CTRLA = ADC_DMASEL_CH0123_gc;
    ADCA.CTRLB = ADC_CURRLIMIT_LARGE_gc | ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;
    ADCA.REFCTRL = ADC_REFSEL_VCC_gc;
    ADCA.EVCTRL = 0;
    ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc;
    ADCA.INTFLAGS = ADC_CH3IF_bm | ADC_CH2IF_bm | ADC_CH1IF_bm | ADC_CH0IF_bm;
    ADCA.CAL = adca_calibration;

    ADCA.CH0.CTRL = ADC_CH_GAIN_DIV2_gc | ADC_CH_INPUTMODE_DIFFWGAIN_gc;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_POTI_BIT_ERROR_RATE_gc | ADC_CH_WGAIN_MUXNEG_PADGND_gc;
    ADCA.CH0.INTCTRL = 0;
    ADCA.CH0.SCAN = 0;

    ADCA.CH1.CTRL = ADC_CH_GAIN_DIV2_gc | ADC_CH_INPUTMODE_DIFFWGAIN_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_POTI_BLOCKING_RATE_gc | ADC_CH_WGAIN_MUXNEG_PADGND_gc;
    ADCA.CH1.INTCTRL = 0;
    ADCA.CH1.SCAN = 0;

    ADCA.CH2.CTRL = ADC_CH_GAIN_DIV2_gc | ADC_CH_INPUTMODE_DIFFWGAIN_gc;
    ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_POTI_BLOCKING_DURATION_gc | ADC_CH_WGAIN_MUXNEG_PADGND_gc;
    ADCA.CH2.INTCTRL = 0;
    ADCA.CH2.SCAN = 0;

    ADCA.CH3.CTRL = ADC_CH_GAIN_DIV2_gc | ADC_CH_INPUTMODE_DIFFWGAIN_gc;
    ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_CURRENT_SENSE_gc | ADC_CH_WGAIN_MUXNEG_PADGND_gc;
    ADCA.CH3.INTCTRL = 0;
    ADCA.CH3.SCAN = 0;

    /*
     * init DMA to store poti and current sense results in buffer while running
     * in background.
     */
    DMA.CTRL = DMA_DBUFMODE_DISABLED_gc | DMA_PRIMODE_RR0123_gc;
    DMA.CH0.CTRLA = DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_8BYTE_gc;
    DMA.CH0.CTRLB = 0;
    DMA.CH0.ADDRCTRL = DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_INC_gc
        | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;
    DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_ADCA_CH0_gc;
    DMA.CH0.TRFCNT = sizeof(adc_sense_buffer);
    DMA.CH0.REPCNT = 0;
    DMA.CH0.CTRLA |= DMA_CH_REPEAT_bm;
    const uint16_t adc_ch0_res = (uint16_t) &ADCA.CH0RES;
    DMA.CH0.SRCADDR0 = (uint8_t) adc_ch0_res;
    DMA.CH0.SRCADDR1 = (uint8_t) (adc_ch0_res >> 8);
    DMA.CH0.SRCADDR2 = 0;
    const uint16_t buffer_addr = (uint16_t) &adc_sense_buffer;
    DMA.CH0.DESTADDR0 = (uint8_t) buffer_addr;
    DMA.CH0.DESTADDR1 = (uint8_t) (buffer_addr >> 8);
    DMA.CH0.DESTADDR2 = 0;

    DMA.CTRL |= DMA_ENABLE_bm;
    DMA.CH0.CTRLA |= DMA_CH_ENABLE_bm;

//XXX    ADCA.CTRLB |= ADC_FREERUN_bm;
    ADCA.CTRLA |= /*XXX ADC_CH0START_bm | */ADC_FLUSH_bm | ADC_ENABLE_bm;
}
