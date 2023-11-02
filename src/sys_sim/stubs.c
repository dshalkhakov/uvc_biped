#include <stdio.h>
#include "kcb5.h"
#include "ad.h"
#include "uart.h"
#include "timer.h"
#include "ics.h"
#include "i2c.h"

bool ad_init(int port, ad_mode_t mode)
{
	return true;
}

void dac_init()
{

}

int dac_write(unsigned short value)
{
	return 0;
}

bool sio_init(int port, int baudrate)
{
	return true;
}

bool pio_init(int port, int direction)
{
	return true;
}

int pio_read(int port)
{
	return 0;
}

int pio_write(int port, int value)
{
	return 0;
}
