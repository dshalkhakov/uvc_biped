#include "kcb5.h"

int (*g_ics_set_pos_callback)(int, unsigned char, unsigned short);

int ics_set_pos(int port, unsigned char id, unsigned short pos)
{
	if (g_ics_set_pos_callback != NULL) {
		return (*g_ics_set_pos_callback)(port, id, pos);
	}
	return 0;
}

int ics_set_param(int port, unsigned char id, unsigned char sc, unsigned char param)
{
	return 0;
}
