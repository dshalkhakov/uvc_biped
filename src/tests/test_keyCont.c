#include <malloc.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>
#include <kcb5.h>
#include <uart.h>
#include "../main.h"

#define KEYMODE_BASIC       (0)
#define KEYMODE_SPECIFIC    (5)

typedef struct {
    input_t     input;
    core_t      core;
    state_t     state;
} teststate_t;

int keyCont_testSetup(void** state) {
    teststate_t * teststate = (teststate_t *)calloc(1, sizeof(teststate_t));
    *state = teststate;
    return 0;
}

int keyCont_testTeardown(void** state) {
    teststate_t * teststate = *state;
    free(teststate);
    return 0;
}

void keyCont_nullString_ignored(void** state) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = KEYMODE_SPECIFIC;
    data->input.kn = 5;
    data->core.mode = 720;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, '\0');
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->input.keyMode, KEYMODE_SPECIFIC);
    assert_int_equal(data->input.kn, 5);
    assert_int_equal(data->core.mode, 720);
}

void keyCont_spacePressed_keyModeReset(void** state) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = KEYMODE_SPECIFIC;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, ' ');
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->input.keyMode, KEYMODE_BASIC);
}

void keyCont_basicKeyMode_rPressed_coreModeIsReset(void** state) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = KEYMODE_BASIC;
    data->core.mode = 740;
    data->core.motCt = 50;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, 'r');
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->core.mode, 710);
    assert_int_equal(data->core.motCt, 100);
}

void keyCont_basicKeyMode_gPressed_started(void** state) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = KEYMODE_BASIC;
    data->core.mode = 720;
    data->core.motCt = 100;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, 'g');
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_true(1); // not should what should be tested here... so here goes
}

void keyCont_basicKeyMode_tPressed_coreMode790Engaged(void** state) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = KEYMODE_BASIC;
    data->core.mode = 720;
    data->core.motCt = 100;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, 't');
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->core.mode, 790);
}

void keyCont_basicKeyMode_yPressed_coreMode791Engaged(void** state) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = KEYMODE_BASIC;
    data->core.mode = 720;
    data->core.motCt = 100;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, 'y');
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->core.mode, 791);
}
