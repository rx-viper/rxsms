#ifndef TASK_H
#define TASK_H

/* a task that starts runs in background */
struct bgtask
{
    void (*init)(void);
};

/* a normal real-time task */
struct task
{
    void (*init)(void);
    void (*run)(void);
};

#endif /* TASK_H */
