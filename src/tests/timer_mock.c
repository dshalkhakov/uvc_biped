#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>

#include "kcb5.h"
#include "timer.h"
#pragma comment(linker,"/alternatename:timer_init=__wrap_timer_init")
#pragma comment(linker,"/alternatename:timer_write=__wrap_timer_write")
#pragma comment(linker,"/alternatename:timer_start=__wrap_timer_start")
#pragma comment(linker,"/alternatename:timer_read=__wrap_timer_read")

bool __wrap_timer_init(int port, timer_mode mode, int frequency) {
	return mock_type(bool);
}

bool __wrap_timer_write(int port, unsigned int data) {
	return mock_type(bool);
}

int __wrap_timer_start(int port) {
	return mock_type(int);
}

unsigned int __wrap_timer_read(int port) {
	return mock_type(unsigned int);
}

#define __wrap_timer_init timer_init
#define __wrap_timer_write timer_write
#define __wrap_timer_start timer_start
#define __wrap_timer_read timer_read
