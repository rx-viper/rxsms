#ifndef TASK_BUTTONS_H
#define TASK_BUTTONS_H

#include "task.h"

const struct task task_buttons;

struct
{
    uint8_t lo : 1;
    uint8_t soe : 1;
    uint8_t sods : 1;
    uint8_t errinh : 1;
    uint8_t pwr : 1;
} task_buttons_toggle_request;


#endif /* TASK_BUTTONS_H */
