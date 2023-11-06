#include <conio.h>
#include <stdio.h>
#include "kcb5.h"
#include "uart.h"
#include "sim_uart.h"

static unsigned char g_lastKeyPressed = '\0';

bool uart_init(int port, uart_mode mode, unsigned int baudrate, unsigned
	int data, unsigned int parity)
{
	return true;
}

bool uart_tx(int port, unsigned char* tx, int start, int length)
{
	if (port == UART_COM)
	{
		printf(tx);
	}
	return true;
}

bool uart_rx(int port, unsigned char* rx, int length, unsigned long timeout)
{
	if (port == UART_COM) {
		if (g_lastKeyPressed != '\0') {
			*rx = g_lastKeyPressed;
			g_lastKeyPressed = '\0';
			return true;
		}
		else {
			*rx = 0;
			return false;
		}
	}
	return true;
}

void simuart_setLastKeyPressed(unsigned char key) {
	g_lastKeyPressed = key;
}
