#include <avr/io.h>
#include "task_debug.h"

static void init(void);
static void run(void);
const struct task task_debug = { .init = &init, .run = &run };

static void
init(void)
{
    PORTD.DIRSET = PIN4_bm;
}

static void
run(void)
{
    static uint16_t i = 0;
    i = (i + 1) % 400;
    if (0 == i)
       PORTD.OUTTGL = PIN4_bm;
}
