#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <signal.h>  
#include <sys/syscall.h>
#include <time.h>

#include "lib/pigpio/pigpio.h"

#include "worker_thread.h"
#include "pthread_defines.h"



#define GPIO_PIN_IN  (20)
#define GPIO_PIN_OUT (21)
//#define GPIO_PIN_PWM (13)
#define GPIO_PIN_PWM (12)

typedef struct user_data_ {
   uint32_t cnt;
   uint32_t len;
   uint32_t *array;
   uint32_t val;
} user_data;

typedef struct ts_data_ {
	struct timespec ts_start;
	struct timespec ts_stop;
} ts_data;

void Handler(int signo)
{
    //System Exit
    printf("Sigint handler worker thread - Exit\n");
	/* Stop DMA, release resources */
	gpioWrite(GPIO_PIN_OUT, PI_LOW);
	gpioTerminate();

    exit(0);
}

void timespec_diff(const struct timespec *start, const struct timespec *stop,
                   struct timespec *result)
{
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }
    return NULL;
}

static void callback(int gpio, int level, uint32_t tick, void *data) {

	//printf("callback\n");
	user_data *p_data = data;
	gpioWrite(GPIO_PIN_OUT, level);

	//return NULL;
	//printf("%u\n", tick);
	if (level == 1)
		p_data->array[(p_data->cnt++)%p_data->len] = tick;

}

static void callback_irq_jitter(int gpio, int level, uint32_t tick, void *data) {

	int do_print = 0;
	double factor = 0.000001;
	static int cnt;
	static int min = 1e9;
	static int max = -1;
	static double mean = 25000.0;

	//printf("callback_irg_jitter\n");
	struct timespec ts, ts_diff;
	//user_data *p_data = data;
	struct timespec *p_data = data;
	//gpioWrite(GPIO_PIN_OUT, level);

	if (level == PI_LOW)
		return NULL;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	timespec_diff(p_data, &ts, &ts_diff);

	if (ts_diff.tv_nsec < min) {
		min = ts_diff.tv_nsec;
		do_print = 1;
	}
	if (ts_diff.tv_nsec > max) {
		max = ts_diff.tv_nsec;
		do_print = 1;
	}
	mean = mean*(1.0-factor) + (double)ts_diff.tv_nsec*factor;
	if (do_print && (cnt > 100)) {
		printf("min ns = %d\n", min);
		printf("max ns = %d\n", max);
		printf("mean ns = %f\n\n", mean);
	}
	if (cnt%10000 == 0) {
		printf("min ns = %d\n", min);
		printf("max ns = %d\n", max);
		printf("mean ns = %f\n\n", mean);
	}
	cnt++;

	gpioWrite(GPIO_PIN_OUT, PI_LOW);
	//printf("time  s: %d\n", ts_diff.tv_sec);
	//printf("jitter us: %d\n", ts_diff.tv_nsec/1000);

	return NULL;
	//printf("%u\n", tick);
	//if (level == 1)
	//	p_data->array[(p_data->cnt++)%p_data->len] = tick;

}

void *thread_func(void *thread_data)
{

	double start;
	user_data data;
	struct timespec ts, ts_start, ts_stop, ts_diff;

	int cfg;
	pid_t tid;

    tid = syscall(SYS_gettid);
    printf("PID %d: Worker thread\n", tid);
    printf("Worker thread is running\n");

	// Exception handling:ctrl + c
    signal(SIGINT, Handler);

	data.cnt = 0;
	data.len = 4;
	data.array = (uint32_t*)malloc(data.len * sizeof(uint32_t));
	memset(data.array, 0, data.len*sizeof(uint32_t));

	//turn off pigpio signal handling
	cfg = gpioCfgGetInternals();
	cfg |= PI_CFG_NOSIGHANDLER;
	gpioCfgSetInternals(cfg);

	//gpioCfgClock(1, 1, 0);
	if (gpioInitialise(PIGPIO_THREAD_PRIORITY, PIGPIO_THREAD_CPU_AFFINITY) < 0)
	{
		fprintf(stderr, "pigpio initialisation failed\n");
		return 1;
	}

    printf("pigpio is initialized\n");

	/* Set GPIO modes */
	gpioSetMode(GPIO_PIN_IN, PI_INPUT);
	gpioSetMode(GPIO_PIN_OUT, PI_OUTPUT);
	gpioSetMode(GPIO_PIN_PWM, PI_OUTPUT);

	gpioSetPullUpDown(GPIO_PIN_IN, PI_PUD_DOWN);
	gpioWrite(GPIO_PIN_OUT, PI_HIGH);
	gpioWrite(GPIO_PIN_OUT, PI_LOW);
    printf("pins are set\n");

	//gpioSetAlertFuncEx(GPIO_PIN_IN, callback, (void*)&data);

	//gpioSetISRFuncEx(GPIO_PIN_IN, EITHER_EDGE, 1000, callback, (void*)&data);
	gpioSetISRFuncEx(GPIO_PIN_IN, RISING_EDGE, 1000, callback_irq_jitter, (void*)&ts);
    printf("isr is set\n");
	time_sleep(0.5);

	/* Start 1500 us servo pulses on GPIO4 */
	//gpioServo(4, 1500);

	/* Start 75% dutycycle PWM on GPIO17 */
	//gpioSetPWMfrequency(GPIO_PIN_PWM, 8000);
	//gpioSetPWMfrequency(GPIO_PIN_OUT, 40000);
	//gpioSetPWMfrequency(GPIO_PIN_PWM, 40000);
	//gpioPWM(GPIO_PIN_PWM, 128); /* 192/255 = 75% */
	//gpioPWM(GPIO_PIN_OUT, 12); /* 192/255 = 75% */
	//gpioPWM(GPIO_PIN_PWM, 192); /* 192/255 = 75% */

	//start = time_time();
	//gpioTerminate();
	//return 0;

	int cnt = 0;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	clock_gettime(CLOCK_MONOTONIC, &ts_start);
    printf("clock_gettime is set\n");

	//while ((time_time() - start) < 60.0)
	while (1)
	{
	//   gpioWrite(18, 1); /* on */

	//   time_sleep(0.5);

	//   gpioWrite(18, 0); /* off */

	//   time_sleep(0.5);

	//   /* Mirror GPIO24 from GPIO23 */
	//   gpioWrite(24, gpioRead(23));
		/*
		printf("ticks [%d]:\n", cnt++);
		for (int i=0; i<data.len; i++) {
			uint32_t ms = data.array[i]/1000;
			uint32_t s  = ms/1000;
			printf("tick: %.3u-", s);
			printf("%.3u-", (data.array[i] - s*1000*1000)/1000);
			printf("%.3u\n", data.array[i] - ms*1000);
		}
		*/
		clock_gettime(CLOCK_MONOTONIC, &ts);
		//timespec_diff(&ts_start, &ts, &ts_diff);
		//printf("time  s: %d\n", ts.tv_sec);
		//printf("time ns: %d\n", ts.tv_nsec);
		//printf("time  s: %d\n", ts_diff.tv_sec);
		//printf("time ns: %d\n", ts_diff.tv_nsec);

		gpioWrite(GPIO_PIN_OUT, PI_HIGH);
		if (cnt == 0) {
    		printf("gpio out is high\n");
		}
		time_sleep(0.0001);
		cnt++;
		//gpioWrite(GPIO_PIN_OUT, PI_LOW);
		//time_sleep(0.1);
		//if (cnt == 10)
		//	break;
	}

	/* Stop DMA, release resources */
	gpioTerminate();



	return NULL;
}