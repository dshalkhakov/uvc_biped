#include <stdio.h>
#include <time.h>
#include "kcb5.h"
#include "timer.h"

// DS: unfortunately, I have no idea how timers should work, but this implementation seems to allow for the program to progress

typedef struct timerState_s {
	long long	started;
	timer_mode	mode;
	int			frequency;
	int			delay;		// in microseconds
} timerState_t;

static timerState_t sys_timers[255];

bool timer_init(int port, timer_mode mode, int frequency)
{
	timerState_t * timer = &sys_timers[port];

	timer->frequency	= frequency;
	timer->mode			= mode;
	timer->started		= 0;

	return true;
}

bool timer_write(int port, unsigned int data)
{
	timerState_t* timer = &sys_timers[port];

	timer->delay = data;

	return true;
}


long long millis()
{
	struct timespec now;
	timespec_get(&now, TIME_UTC);
	return ((long long)now.tv_sec) * 1000 + ((long long)now.tv_nsec) / 1000000;
}

int timer_start(int port)
{
	timerState_t* timer = &sys_timers[port];
	int delayMs = timer->delay * 0.001;

	timer->started = millis();

	return 1;
}

unsigned int timer_read(int port)
{
	timerState_t* timer = &sys_timers[port];
	int delayMs = timer->delay * 0.001;

	long long current = millis();
	if ((current - timer->started) > delayMs)
	{
		return -1;
	}

	return 0;
}
