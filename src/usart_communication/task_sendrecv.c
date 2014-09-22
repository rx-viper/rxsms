#include <avr/io.h>
#include "task_sendrecv.h"

#define STATUS_LED_PORT         PORTD
#define STATUS_LED_UPLINK_bm    PIN4_bm
#define STATUS_LED_DOWNLINK_bm  PIN5_bm

#define UART_PORT                   PORTD
#define UART_GROUNDSTATION_RX_bm    PIN6_bm
#define UART_GROUNDSTATION_TX_bm    PIN7_bm
#define UART_EXPERIMENT_RX_bm       PIN2_bm
#define UART_EXPERIMENT_TX_bm       PIN3_bm

#define UART_GROUNDSTATION  USARTD1
#define UART_EXPERIMENT     USARTD0
#define UART_BSEL       (3269)
#define UART_BSCALE     (-6)

#define LED_DURATION    400

static void init(void);
static void recv(void);
static void send(void);
/* common init routine for both as they access the same hardware */
const struct task task_recv = { .init = &init, .run = &recv };
const struct task task_send = { .init = &init, .run = &send };

static struct uart_data
{
    uint8_t data;
    uint8_t updated : 1;
    uint16_t inactivity;
    uint16_t led_toggle_interval;
} from_gnd, from_exp;

static void
init(void)
{
    STATUS_LED_PORT.OUTCLR = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;
    STATUS_LED_PORT.DIRSET = STATUS_LED_UPLINK_bm | STATUS_LED_DOWNLINK_bm;

    UART_PORT.OUTSET = UART_GROUNDSTATION_TX_bm | UART_EXPERIMENT_TX_bm;
    UART_PORT.DIRSET = UART_GROUNDSTATION_TX_bm | UART_EXPERIMENT_TX_bm;
    UART_PORT.DIRCLR = UART_GROUNDSTATION_RX_bm | UART_EXPERIMENT_RX_bm;

    UART_GROUNDSTATION.CTRLA = 0;
    UART_GROUNDSTATION.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    UART_GROUNDSTATION.CTRLC = USART_CMODE_ASYNCHRONOUS_gc
        | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
    UART_GROUNDSTATION.BAUDCTRLA = (uint8_t) UART_BSEL;
    UART_GROUNDSTATION.BAUDCTRLB = (UART_BSCALE << 4) | (UART_BSEL >> 8);

    UART_EXPERIMENT.CTRLA = 0;
    UART_EXPERIMENT.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    UART_EXPERIMENT.CTRLC = USART_CMODE_ASYNCHRONOUS_gc
        | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
    UART_EXPERIMENT.BAUDCTRLA = (uint8_t) UART_BSEL;
    UART_EXPERIMENT.BAUDCTRLB = (UART_BSCALE << 4) | (UART_BSEL >> 8);

    const struct uart_data initializer = { .data = 0, .updated = 0,
        .inactivity = LED_DURATION, .led_toggle_interval = LED_DURATION / 2 };
    from_gnd = initializer;
    from_exp = initializer;
}

static void
recv_uart(USART_t *uart, struct uart_data *data)
{
    --data->inactivity;
    uint8_t err_flags = USART_FERR_bm | USART_BUFOVF_bm | USART_PERR_bm;
    if (uart->STATUS & err_flags) {
        /* clear error flags if set */
        uart->STATUS |= err_flags;
        return;
    }

    if (!(uart->STATUS & USART_RXCIF_bm))
        return;

    data->data = uart->DATA;
    data->updated = 1;
    data->inactivity = LED_DURATION;
    --data->led_toggle_interval;
}

static void
update_led(struct uart_data *data, uint8_t ledmask)
{
    if (0 == data->inactivity) {
        /* we have not seen any activity for some time, timeout */
        data->inactivity = LED_DURATION;
        STATUS_LED_PORT.OUTCLR = ledmask;
    } else {
        if (0 == data->led_toggle_interval) {
            /* a normal period of activity passed */
            data->led_toggle_interval = LED_DURATION / 2;
            STATUS_LED_PORT.OUTTGL = ledmask;
        }
    }
}


static void
send_uart(USART_t *uart, struct uart_data *data)
{
    if (!data->updated)
        return; /* nothing to send */

    if (!(uart->STATUS & USART_DREIF_bm))
        return; /* cannot send, USART busy, drop byte */

    // TODO apply error pattern
    // TODO use ERRINH state
    uart->DATA = data->data;
    data->updated = 0;
}
static void
recv(void)
{
    recv_uart(&UART_EXPERIMENT, &from_exp);
    recv_uart(&UART_GROUNDSTATION, &from_gnd);
    update_led(&from_exp, STATUS_LED_DOWNLINK_bm);
    update_led(&from_gnd, STATUS_LED_UPLINK_bm);
}

static void
send(void)
{
    send_uart(&UART_GROUNDSTATION, &from_exp);
    // TODO disable forwarding uplink when LO given
    // if (enabled) {
    send_uart(&UART_EXPERIMENT, &from_gnd);
    // }
}
