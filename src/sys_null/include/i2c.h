#include <stddef.h>

typedef enum
{
	I2C_MASTER,
	I2C_SLAVE
} i2c_mode;

// Function Initializes the i2c port.
// Arguments int clock Specifies the clock for the device.For example, for 400kHz, specify 400000.
// enum mode specify I2C_MASTER to use as master, I2C_SLAVE to use as slave
// Only return value true is returned.
extern bool i2c_init(int clock, i2c_mode mode);

// Function Read data from i2c port
// Argument int i2c_address i2c address
// byte* command Command array required to read data
// size_t c_size Data command size
// byte* data Array that stores the read data
// size_t r_size Read data size
// Return value Always returns 0.
// Remarks If there is no response, such as when there is no device, the function will continue to wait inside the function.
extern int i2c_read(int i2c_address, unsigned char* command, size_t c_size, unsigned char* data, size_t r_size);

// Function Writes data from i2c port.
// Argument unsigned char i2c_address i2c address
// int address i2c device address
// byte* data Write data
// size_t size　Data size for writing
// Return value Always returns 0.
// Remarks If there is no response, such as when there is no device, the function will continue to wait inside the function.
extern int i2c_write(unsigned char i2c_address, unsigned char address, unsigned char* data, size_t size);
