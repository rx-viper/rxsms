#include <avr/io.h>
#include "task_buttons.h"

/*
 * Ressources used _once_ by this task:
 * (none)
 *
 * Ressources _continously_ used by this task:
 * BUTTON_PORT: BUTTON_LO_bp BUTTON_SOE_bp BUTTON_SODS_bp BUTTON_ERRINH_bp
 * BUTTON_EXTENDED_PORT: BUTTON_EXTENDED_PWR_bp
 */

#define BUTTON_PORT         PORTB
#define BUTTON_LO_bp        PIN0_bp
#define BUTTON_SOE_bp       PIN1_bp
#define BUTTON_SODS_bp      PIN2_bp
#define BUTTON_ERRINH_bp    PIN3_bp

#define BUTTON_EXTENDED_PORT    PORTC
#define BUTTON_EXTENDED_PWR_bp  PIN1_bp

/// The minimum time a button must be pressed to be counted as valid signal
#define DURATION_DEBOUNCE    100

static void init(void);
static void run(void);

const struct task task_buttons = { .init = &init, .run = &run };

struct button
{
    uint8_t pressed;
    uint8_t released;
    uint8_t triggered;
};

static struct button lo;
static struct button soe;
static struct button sods;
static struct button errinh;
static struct button pwr;

static void
init(void)
{
    BUTTON_PORT.DIRCLR = _BV(BUTTON_LO_bp) | _BV(BUTTON_SOE_bp)
        | _BV(BUTTON_SODS_bp) | _BV(BUTTON_ERRINH_bp);
    BUTTON_EXTENDED_PORT.DIRCLR = _BV(BUTTON_EXTENDED_PWR_bp);

    /* 
     * must be done in two steps to allow resolving of num in PINCTRL first,
     * before doing the actual conatenation/token pasting; otherwise we would
     * get something like PINBUTTON_SOE_bpCTRL, which is bullshit!
     */
#define PINCTRL_CONCAT(num) PIN##num##CTRL
#define PINCTRL(num)        PINCTRL_CONCAT(num)
    BUTTON_PORT.PINCTRL(BUTTON_LO_bp) = PORT_OPC_PULLUP_gc;
    BUTTON_PORT.PINCTRL(BUTTON_SOE_bp) = PORT_OPC_PULLUP_gc;
    BUTTON_PORT.PINCTRL(BUTTON_SODS_bp) = PORT_OPC_PULLUP_gc;
    BUTTON_PORT.PINCTRL(BUTTON_ERRINH_bp) = PORT_OPC_PULLUP_gc;
    BUTTON_EXTENDED_PORT.PINCTRL(BUTTON_EXTENDED_PWR_bp) = PORT_OPC_PULLUP_gc;
#undef PINCTRL
#undef PINCTRL_CONCAT

    const struct button initializer = {
        .pressed = DURATION_DEBOUNCE,
        .released = DURATION_DEBOUNCE,
        .triggered = 0
    };
    lo = initializer;
    soe = initializer;
    sods = initializer;
    errinh = initializer;
    pwr = initializer;
}

static void
update_duration(uint8_t current, struct button * btn)
{
    if (current) {
        if (btn->released) /* prevent underflow when already 0 */
            --btn->released;
        btn->pressed = DURATION_DEBOUNCE;
    } else {
        /* decrease other counter only, iff we were back to normal (released)
         * state for long enough */
        if (!btn->released) {
            /* prevent underflow when already 0 */
            if (btn->pressed) {
                --btn->pressed;
            } else {
                /* we have a valid low signal, so let's reset the high time */
                btn->triggered = 1;
                btn->released = DURATION_DEBOUNCE;
            }
        }
    }
}

static void
run(void)
{
    const uint8_t port = BUTTON_PORT.IN;
    const uint8_t extended_port = BUTTON_EXTENDED_PORT.IN;

    update_duration(port & _BV(BUTTON_LO_bp), &lo);
    update_duration(port & _BV(BUTTON_SOE_bp), &soe);
    update_duration(port & _BV(BUTTON_SODS_bp), &sods);
    update_duration(port & _BV(BUTTON_ERRINH_bp), &errinh);
    update_duration(extended_port & _BV(BUTTON_EXTENDED_PWR_bp), &pwr);

    /* check if stable, and thus, valid */
    if (lo.triggered) {
        task_buttons_toggle_request.lo = 1;
        lo.triggered = 0;
    } else {
        task_buttons_toggle_request.lo = 0;
    }
    if (soe.triggered) {
        task_buttons_toggle_request.soe = 1;
        soe.triggered = 0;
    } else {
        task_buttons_toggle_request.soe = 0;
    }
    if (sods.triggered) {
        task_buttons_toggle_request.sods = 1;
        sods.triggered = 0;
    } else {
        task_buttons_toggle_request.sods = 0;
    }
    if (errinh.triggered) {
        task_buttons_toggle_request.errinh = 1;
        errinh.triggered = 0;
    } else {
        task_buttons_toggle_request.errinh = 0;
    }
    if (pwr.triggered) {
        task_buttons_toggle_request.pwr = 1;
        pwr.triggered = 0;
    } else {
        task_buttons_toggle_request.pwr = 0;
    }
}
