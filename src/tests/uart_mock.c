#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>

#include "kcb5.h"
#include "uart.h"
#pragma comment(linker,"/alternatename:uart_init=__wrap_uart_init")
#pragma comment(linker,"/alternatename:uart_tx=__wrap_uart_tx")
#pragma comment(linker,"/alternatename:uart_rx=__wrap_uart_rx")

bool __wrap_uart_init(int port, uart_mode mode, unsigned int baudrate, unsigned
	int data, unsigned int parity) {
	bool ret;

	check_expected(port);
	check_expected(mode);
	check_expected(baudrate);
	check_expected(data);
	check_expected(parity);

	ret = mock_type(bool);

	return ret;
}

bool __wrap_uart_tx(int port, unsigned char* tx, int start, int length) {
	return true;
}

bool __wrap_uart_rx(int port, unsigned char* rx, int length, unsigned long timeout) {
	bool ret;

	check_expected(port);
	if (rx != NULL)
	{
		*rx = mock_ptr_type(unsigned char*);
	}
	check_expected(length);
	check_expected(timeout);

	ret = mock_type(bool);

	return ret;
}
