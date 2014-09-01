#ifndef TASK_CTRL_H
#define TASK_CTRL_H

#include <inttypes.h>
#include "task.h"

const struct task task_ctrl;
/// if 1, Inhibit bit error generation, else create bit errors as usual.
uint8_t task_ctrl_is_error_inhibit;

#endif /* TASK_CTRL_H */
