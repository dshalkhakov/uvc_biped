#include "kcb5.h"
#include "ad.h"
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

bool i2c_init(int clock, i2c_mode mode)
{
	return true;
}

int i2c_read(int i2c_address, unsigned char* command, size_t c_size, unsigned char* data, size_t r_size)
{
	return 0;
}

int i2c_write(unsigned char i2c_address, unsigned char address, unsigned char* data, size_t size)
{
	return 0;
}

int ics_set_pos(int port, unsigned char id, unsigned short pos)
{
	return 0;
}

int ics_set_param(int port, unsigned char id, unsigned char sc, unsigned char param)
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
