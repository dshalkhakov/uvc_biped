#include "kcb5.h"
#include "i2c.h"

//================ BNO55 stubs ===================

#define BNO055_ID	0xA0

static bool g_bno55_initialized = false;
int (*g_bno55_read)(unsigned char* command, size_t c_size, unsigned char* data, size_t r_size);

int bno55_read(unsigned char* command, size_t c_size, unsigned char* data, size_t r_size) {
	if (c_size == 1 && command[0] == 0 && r_size == 1) {
		// initialized at first call. this is to force main to wait for it to boot.
		if (g_bno55_initialized == false) {
			g_bno55_initialized = true;
			return 0;
		}
		else
		{
			data[0] = BNO055_ID;
			return 1;
		}
	}
	else if (c_size == 1 && command[0] == 0X1A && r_size == 6) {
		// return gyroscope data
		// core->yaw	= ((int16_t)input->ff[0]) | (((int16_t)input->ff[1]) << 8); // Standing upright and turning clockwise +
		// core->pitchs = ((int16_t)input->ff[2]) | (((int16_t)input->ff[3]) << 8); // Standing upright and leaning forward -
		// core->rolls = ((int16_t)input->ff[4]) | (((int16_t)input->ff[5]) << 8); // Upright with right tilt +
		data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 0;
		if (g_bno55_read != NULL) {
			return (*g_bno55_read)(command, c_size, data, r_size);
		}
		else {
			return 1;
		}
	}
	else if (c_size == 1 && command[0] == 0X14 && r_size == 6) {
		// return angular velocity
		data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 0;
		if (g_bno55_read != NULL) {
			return (*g_bno55_read)(command, c_size, data, r_size);
		}
		else {
			return 1;
		}
	}
	return 0;
}

//================ I2C stubs ===================

bool i2c_init(int clock, i2c_mode mode)
{
	return true;
}

int i2c_read(int i2c_address, unsigned char* command, size_t c_size, unsigned char* data, size_t r_size)
{
	// when address is 0x50 then it's BNO55
	if (i2c_address == 0x50) {
		return bno55_read(command, c_size, data, r_size);
	}
	return 0;
}

int i2c_write(unsigned char i2c_address, unsigned char address, unsigned char* data, size_t size)
{
	return 0;
}

