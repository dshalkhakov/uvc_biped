#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>

#include "kcb5.h"
#include "i2c.h"
#pragma comment(linker,"/alternatename:i2c_read=__wrap_i2c_read")

int __wrap_i2c_read(int i2c_address, unsigned char* command, size_t c_size, unsigned char* data, size_t r_size)
{
	int ret;

	check_expected(i2c_address);
	check_expected_ptr(command);
	check_expected(c_size);
	if (data != NULL)
	{
		*data = mock_ptr_type(unsigned char*);
	}
	check_expected(r_size);

	ret = mock_type(int);

	return ret;
}
