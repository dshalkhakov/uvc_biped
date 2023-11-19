#ifdef USING_MAIN

extern "C" {
#include <kcb5.h>
#include <stdio.h>
#include <uart.h>
#include "../src/main.h"
#include "../src/sys_sim/sim_ics.h"
#include "../src/sys_sim/sim_i2c.h"
#include "../src/sys_sim/sim_uart.h"
}

#include <ode/ode.h>
#include "biped.h"

state_t	g_mainstate;
input_t	g_maininput;
core_t	g_maincore;
int16_t g_main_initialI;

void main_integration_feed_simstate(simstate_t* simstate) {
	simstate->K0W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K0W[0]));
	simstate->K0W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K0W[1]));
	simstate->K1W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K1W[0]));
	simstate->K1W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K1W[1]));
	simstate->K2W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K2W[0]));
	simstate->K2W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.K2W[1]));
	simstate->HW[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.HW[0]));
	simstate->HW[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.HW[1]));
	simstate->A0W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A0W[0]));
	simstate->A0W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A0W[1]));
	simstate->A1W[0] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A1W[0]));
	simstate->A1W[1] = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.A1W[1]));
	simstate->WESTW = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.WESTW));
	simstate->HEADW = DEGREES2RADIANS(SVANGLE2ANGLE(g_mainstate.HEADW));
}

void main_integration_command(int cmd) {
	simuart_setLastKeyPressed(cmd);

	switch (cmd) {
	case 'g':case 'G':
		printf("=============== main walk\n");
		g_maincore.mode = 740; // switch to walking
		// g_maincore.dxi += 3; // turns left
		// g_maincore.dxi -= 3; // should turn right? but actually turns left
		// dyi = 10 -> biped spreads legs to sides to increase support area
		break;
	default:
		break;
	}
}

void main_integration_simLoop() {
	// map input to maininput
	// map core to maincore
	g_main_initialI = main_step(&g_mainstate, &g_maincore, &g_maininput, g_main_initialI);
}

void main_integration_restart() {
	g_main_initialI = main_init(&g_mainstate, &g_maincore, &g_maininput);
	g_maincore.mode = 710;
}

int sim_ics_set_pos(int port, unsigned char id, unsigned short pos) {
	simstate_t* state = &simstate;
	double* dest = NULL;

	switch (port) {
	case UART_SIO1:
		switch (id) {
		case 5:  dest = &state->K2W[0]; break;
		case 6:  dest = &state->K1W[0]; break;
		case 7:  dest = &state->K0W[0]; break;
		case 8:  dest = &state->HW[0]; break;
		case 9:  dest = &state->A0W[0]; break;
		case 10: dest = &state->A1W[0]; break;
		default: break;
		}
		break;

	case UART_SIO2:
		switch (id) {
		case 1: dest = &state->U0W[0]; break;
		case 2: dest = &state->U1W[0]; break;
		case 3: dest = &state->U2W[0]; break;
		case 4: /*dest = &state->EW[0];*/ break;
		case 0: dest = &state->WESTW; break;
		default: break;
		}
		break;

	case UART_SIO3:
		switch (id) {
		case 5:  dest = &state->K2W[1]; break;
		case 6:  dest = &state->K1W[1]; break;
		case 7:  dest = &state->K0W[1]; break;
		case 8:  dest = &state->HW[1]; break;
		case 9:  dest = &state->A0W[1]; break;
		case 10: dest = &state->A1W[1]; break;
		default: break;
		}
		break;

	case UART_SIO4:
		switch (id) {
		case 1: dest = &state->U0W[1]; break;
		case 2: dest = &state->U1W[1]; break;
		case 3: dest = &state->U2W[1]; break;
			//case 4: dest = &state->EW[1]; break;
		case 0: dest = &state->HEADW; break;
		default: break;
		}
		break;

	default:
		break;
	}

	radangle_t	angle = DEGREES2RADIANS(SVANGLE2ANGLE(-pos + 7500));

	if (dest != NULL) {
		if (pos == 0) {
			return -ANGLE2SVANGLE(RADIANS2DEGREES(*dest)) + 7500;
		}
		else {
			*dest = (double)angle;
			return pos;
		}
	}

	return pos;
}

#define LOW_BYTE(x)		((x) & 0xFF)
#define HIGH_BYTE(x)	(((x) >> 8) & 0xFF)

int sim_bno55_read(unsigned char* command, size_t c_size, unsigned char* data, size_t r_size) {
	if (c_size == 1 && command[0] == 0X1A && r_size == 6) {
		// return gyroscope absolute angle readings as degrees*16.0
		// core->yaw	= ((int16_t)input->ff[0]) | (((int16_t)input->ff[1]) << 8); // Standing upright and turning clockwise +
		// core->pitchs = ((int16_t)input->ff[2]) | (((int16_t)input->ff[3]) << 8); // Standing upright and leaning forward -
		// core->rolls = ((int16_t)input->ff[4]) | (((int16_t)input->ff[5]) << 8); // Upright with right tilt +
		// euler angles to ode 3x4 matrix procedure seems to have roll, pitch, yaw (Z, X, Y) convention
		hwangle_t roll = (RADIANS2DEGREES(simstate.heading[0] + M_PI)) * 16.0,
			pitch = (RADIANS2DEGREES(simstate.heading[1])) * 16.0,
			yaw = (RADIANS2DEGREES(simstate.heading[2])) * 16.0;
		((uint8_t*)data)[0] = LOW_BYTE(yaw);
		((uint8_t*)data)[1] = HIGH_BYTE(yaw);
		((uint8_t*)data)[2] = LOW_BYTE(pitch);
		((uint8_t*)data)[3] = HIGH_BYTE(pitch);
		((uint8_t*)data)[4] = LOW_BYTE(roll);
		((uint8_t*)data)[5] = HIGH_BYTE(roll);
		return 1;
	}
	else if (c_size == 1 && command[0] == 0X14 && r_size == 6) {
		// return angular velocity in radians*1000
		int16_t fbAV = (simstate.fbAV) * 1000.0f;
		int16_t lrAV = (simstate.lrAV) * 1000.0f;
		((uint8_t*)data)[0] = LOW_BYTE(fbAV);	// roll
		((uint8_t*)data)[1] = HIGH_BYTE(fbAV);	// roll
		((uint8_t*)data)[2] = LOW_BYTE(lrAV);	// pitch
		((uint8_t*)data)[3] = HIGH_BYTE(lrAV);	// pitch
		((uint8_t*)data)[4] = 0;	// yaw
		((uint8_t*)data)[5] = 0;	// yaw
		return 1;
	}

	return 1;
}

void main_integration_init() {
	g_ics_set_pos_callback = &sim_ics_set_pos;
	g_bno55_read = &sim_bno55_read;
}

#endif // !USING_MAIN
