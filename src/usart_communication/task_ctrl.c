#include <avr/io.h>
#include "task_ctrl.h"
#include "task_buttons.h"

#define STATUS_LED_PORT         PORTE
#define STATUS_LED_SODS_bm      PIN0_bm
#define STATUS_LED_SOE_bm       PIN1_bm
#define STATUS_LED_LO_bm        PIN2_bm
#define STATUS_LED_ERRINH_bm    PIN3_bm

#define STATUS_LED_EXTENDED_PORT    PORTC
#define STATUS_LED_EXTENDED_PWR_bm  PIN0_bm

#define RXSM_CTRL_PORT      PORTA
#define RXSM_CTRL_LO_bm     PIN5_bm
#define RXSM_CTRL_SOE_bm    PIN6_bm
#define RXSM_CTRL_SODS_bm   PIN7_bm

#define PWRSUPPLY_CTRL_PORT     PORTD
#define PWRSUPPLY_CTRL_PWRSW_bm PIN1_bm

static void init(void);
static void run(void);
const struct task task_ctrl = { .init = &init, .run = &run };

static struct rxsm_ctrl_state
{
    uint8_t lo_active : 1;
    uint8_t soe_active : 1;
    uint8_t sods_active : 1;
    uint8_t pwr_on : 1;
} state;

static void
apply_state(void)
{
    if (state.lo_active) {
        STATUS_LED_PORT.OUTSET = STATUS_LED_LO_bm;
        RXSM_CTRL_PORT.OUTCLR = RXSM_CTRL_LO_bm;
    } else {
        STATUS_LED_PORT.OUTCLR = STATUS_LED_LO_bm;
        RXSM_CTRL_PORT.OUTSET = RXSM_CTRL_LO_bm;
    }
    if (state.soe_active) {
        STATUS_LED_PORT.OUTSET = STATUS_LED_SOE_bm;
        RXSM_CTRL_PORT.OUTCLR = RXSM_CTRL_SOE_bm;
    } else {
        STATUS_LED_PORT.OUTCLR = STATUS_LED_SOE_bm;
        RXSM_CTRL_PORT.OUTSET = RXSM_CTRL_SOE_bm;
    }
    if (state.sods_active) {
        STATUS_LED_PORT.OUTSET = STATUS_LED_SODS_bm;
        RXSM_CTRL_PORT.OUTCLR = RXSM_CTRL_SODS_bm;
    } else {
        STATUS_LED_PORT.OUTCLR = STATUS_LED_SODS_bm;
        RXSM_CTRL_PORT.OUTSET = RXSM_CTRL_SODS_bm;
    }
    if (task_ctrl_is_error_inhibit)
        STATUS_LED_PORT.OUTSET = STATUS_LED_ERRINH_bm;
    else
        STATUS_LED_PORT.OUTCLR = STATUS_LED_ERRINH_bm;
    if (state.pwr_on) {
        STATUS_LED_EXTENDED_PORT.OUTSET = STATUS_LED_EXTENDED_PWR_bm;
        PWRSUPPLY_CTRL_PORT.OUTSET = PWRSUPPLY_CTRL_PWRSW_bm;
    } else {
        STATUS_LED_EXTENDED_PORT.OUTCLR = STATUS_LED_EXTENDED_PWR_bm;
        PWRSUPPLY_CTRL_PORT.OUTCLR = PWRSUPPLY_CTRL_PWRSW_bm;
    }
}

static void
init(void)
{
    /* by default, the error inhibit is ON, all others OFF/INACTIVE */
    struct rxsm_ctrl_state initializer = { .lo_active = 0,
        .soe_active = 0, .sods_active = 0, .pwr_on = 0 };
    state = initializer;
    task_ctrl_is_error_inhibit = 1;

    /* first set the outputs, then the directions.
     * this way, we can ensure that the outputs are undriven/passive but the
     * OUTPUT register is already set correctly. Then the pins are switched to
     * drive loads, and already have the correct output value. */
    apply_state();
    STATUS_LED_PORT.DIRSET = STATUS_LED_LO_bm | STATUS_LED_SOE_bm
        | STATUS_LED_SODS_bm | STATUS_LED_ERRINH_bm;
    STATUS_LED_EXTENDED_PORT.DIRSET = STATUS_LED_EXTENDED_PWR_bm;
    RXSM_CTRL_PORT.DIRSET = RXSM_CTRL_LO_bm | RXSM_CTRL_SOE_bm
        | RXSM_CTRL_SODS_bm;
    PWRSUPPLY_CTRL_PORT.DIRSET = PWRSUPPLY_CTRL_PWRSW_bm;
}

static void
run(void)
{
    /* TODO: at a later point in time, we don't want to use
     * the button requests directly. instead task_sync should
     * send the button requests to others and should receive
     * requests as well, and finally when all simulators are
     * synchronized, we use the requests from task_sync to
     * synchronously switch the signals for all simulators.
     */
    /* update our internal state */
    if (task_buttons_toggle_request.lo)
        state.lo_active ^= 1;
    if (task_buttons_toggle_request.soe)
        state.soe_active ^= 1;
    if (task_buttons_toggle_request.sods)
        state.sods_active ^= 1;
    if (task_buttons_toggle_request.errinh)
        task_ctrl_is_error_inhibit ^= 1;
    if (task_buttons_toggle_request.pwr)
        state.pwr_on ^= 1;

    apply_state();
}

