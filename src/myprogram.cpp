#include <iostream>
#include <cmath>
#include <unistd.h>
#include <time.h>
#include <alchemy/task.h>

#define PERIOD_NS 1000000   // 1 ms period

RT_TASK task;   // Xenomai real-time task

void periodic_task(void *)
{
    timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    for (int i = 0; i < 10000000; i++)
    {
        // Do some computational work
        double x = sin(i);

        // Sleep until the next execution time
        next_time.tv_nsec += PERIOD_NS;
        if (next_time.tv_nsec >= 1000000000)
        {
            next_time.tv_sec += 1;
            next_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
}

int main()
{
    // Create a Xenomai real-time task with default priority and mode
    rt_task_create(&task, "periodic_task", 0, 50, T_JOINABLE);

    // Start the task
    rt_task_start(&task, &periodic_task, NULL);

    // Wait for the task to finish
    rt_task_join(&task);

    return 0;
}
