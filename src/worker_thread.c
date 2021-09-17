#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <signal.h>  

#include <pigpio.h>

#include "worker_thread.h"



#define GPIO_PIN_IN  (20)
#define GPIO_PIN_OUT (21)
//#define GPIO_PIN_PWM (13)
#define GPIO_PIN_PWM (12

void Handler(int signo)
{
    //System Exit
    printf("Sigint handler - Exit\n");
	/* Stop DMA, release resources */
	gpioTerminate();

    exit(0);
}

typedef struct user_data_ {
   uint32_t cnt;
   uint32_t len;
   uint32_t *array;
   uint32_t val;
} user_data;

static void callback(int gpio, int level, uint32_t tick, void *data) {

   //printf("callback\n");
   user_data *p_data = data;
   //gpioWrite(GPIO_PIN_OUT, level);

   //printf("%u\n", tick);
   if (level == 1)
      p_data->array[(p_data->cnt++)%p_data->len] = tick;
   //p_data->array[0] = 5;
   //p_data->val = tick;

}


void *thread_func(void *thread_data)
{

    printf("rt thread is running\n");
	double start;
	user_data data;

	// Exception handling:ctrl + c
    signal(SIGINT, Handler);

	data.cnt = 0;
	data.len = 16;
	data.array = (uint32_t*)malloc(data.len * sizeof(uint32_t));
	memset(data.array, 0, data.len*sizeof(uint32_t));

	//gpioCfgClock(1, 1, 0);
	if (gpioInitialise() < 0)
	{
		fprintf(stderr, "pigpio initialisation failed\n");
		return 1;
	}

    printf("pigpio is initialized\n");

	/* Set GPIO modes */
	gpioSetMode(GPIO_PIN_IN, PI_INPUT);
	gpioSetMode(GPIO_PIN_OUT, PI_OUTPUT);
	//gpioSetMode(GPIO_PIN_PWM, PI_OUTPUT);

	gpioSetPullUpDown(GPIO_PIN_IN, PI_PUD_DOWN);

	gpioSetAlertFuncEx(GPIO_PIN_IN, callback, (void*)&data);

	//gpioSetISRFuncEx(GPIO_PIN_IN, EITHER_EDGE, 1000, callback, NULL);

	/* Start 1500 us servo pulses on GPIO4 */
	//gpioServo(4, 1500);

	/* Start 75% dutycycle PWM on GPIO17 */
	//gpioSetPWMfrequency(GPIO_PIN_OUT, 8000);
	gpioSetPWMfrequency(GPIO_PIN_OUT, 40000);
	//gpioSetPWMfrequency(GPIO_PIN_PWM, 40000);
	gpioPWM(GPIO_PIN_OUT, 255/2); /* 192/255 = 75% */
	//gpioPWM(GPIO_PIN_PWM, 192); /* 192/255 = 75% */

	//start = time_time();
	//gpioTerminate();
	//return 0;

	int cnt = 0;

	//while ((time_time() - start) < 60.0)
	while (1)
	{
	//   gpioWrite(18, 1); /* on */

	//   time_sleep(0.5);

	//   gpioWrite(18, 0); /* off */

	//   time_sleep(0.5);

	//   /* Mirror GPIO24 from GPIO23 */
	//   gpioWrite(24, gpioRead(23));
		time_sleep(1);
		printf("ticks [%d]:\n", cnt++);
		for (int i=0; i<data.len; i++) {
			uint32_t ms = data.array[i]/1000;
			uint32_t s  = ms/1000;
			printf("tick: %.3u-", s);
			printf("%.3u-", (data.array[i] - s*1000*1000)/1000);
			printf("%.3u\n", data.array[i] - ms*1000);
		}

		//if (cnt == 10)
		//	break;
	}

	/* Stop DMA, release resources */
	gpioTerminate();



	return NULL;
}