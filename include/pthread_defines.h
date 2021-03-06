#ifndef PTHREAD_DEFINES_
#define PTHREAD_DEFINES_

#include <pthread.h>

#define WORKER_THREAD_PRIORITY (80)
#define WORKER_THREAD_CPU_AFFINITY (3)
#define WORKER_THREAD_STACK_SIZE (PTHREAD_STACK_MIN)

#define PIGPIO_THREAD_PRIORITY (70)
#define PIGPIO_THREAD_CPU_AFFINITY (3)

#endif // PTHREAD_DEFINES_