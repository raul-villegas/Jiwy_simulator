#include <iostream>
#include <chrono>
#include <thread>
#include <time.h>

void *periodic_task(void *)
{
    timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    timespec prev_end_time;
    clock_gettime(CLOCK_MONOTONIC, &prev_end_time);

    for (int i = 0; i < 10000000; i++)
    {
        // Do some computational work
        double x = i * i;

        timespec iteration_end_time;
        clock_gettime(CLOCK_MONOTONIC, &iteration_end_time);

        double iteration_duration = (iteration_end_time.tv_sec - prev_end_time.tv_sec) * 1000.0 + (iteration_end_time.tv_nsec - prev_end_time.tv_nsec) / 1000000.0;
        std::cout << "Iteration " << i << " duration: " << iteration_duration << " ms" << std::endl;

        prev_end_time = iteration_end_time;

        // Sleep until the next execution time
        next_time.tv_nsec += 1000000000; // 1 ms
        if (next_time.tv_nsec >= 1000000000)
        {
            next_time.tv_sec += 1;
            next_time.tv_nsec -= 1000000000;
        }
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    return NULL;
}

int main()
{
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, &periodic_task, NULL);
    pthread_join(thread_id, NULL);
    return 0;
}