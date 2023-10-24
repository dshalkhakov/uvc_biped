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
    assert_true(1); // not sure should what should be tested here... so here goes
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

void keyCont_basicKeyMode_pPressed_keyMode5Engaged(void** state) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = KEYMODE_BASIC;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, 'p');
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->input.keyMode, 5);
}

#pragma region keyMode 5 digit presses

static void keyCont_keyMode5_digitsPressed_knSet(void** state, char digit, int expectedKn) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = 5;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, digit);
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->input.kn, expectedKn);
    assert_int_equal(data->input.keyMode, 5);
}

void keyCont_keyMode5_digit0Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '0', 0);
}

void keyCont_keyMode5_digit1Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '1', 1);
}

void keyCont_keyMode5_digit2Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '2', 2);
}

void keyCont_keyMode5_digit3Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '3', 3);
}

void keyCont_keyMode5_digit4Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '4', 4);
}

void keyCont_keyMode5_digit5Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '5', 5);
}

void keyCont_keyMode5_digit6Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '6', 6);
}

void keyCont_keyMode5_digit7Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '7', 7);
}

void keyCont_keyMode5_digit8Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '8', 8);
}

void keyCont_keyMode5_digit9Pressed_knSet(void** state) {
    keyCont_keyMode5_digitsPressed_knSet(state, '9', 9);
}

#pragma endregion // !keyMode 5 digit presses

#pragma region core_t getters/setters

void core_dxiSetter(core_t* core, float val) { core->dxi = val; }
float core_dxiGetter(core_t* core) { return core->dxi; }

void core_dyiSetter(core_t* core, float val) { core->dyi = val; }
float core_dyiGetter(core_t* core) { return core->dyi; }

void core_swMaxSetter(core_t* core, float val) { core->swMax = val; }
float core_swMaxGetter(core_t* core) { return core->swMax; }

void core_pitchGyrgSetter(core_t* core, float val) { core->pitch_gyrg = val; }
float core_pitchGyrgGetter(core_t* core) { return core->pitch_gyrg; }

void core_rollGyrgSetter(core_t* core, float val) { core->roll_gyrg = val; }
float core_rollGyrgGetter(core_t* core) { return core->roll_gyrg; }

void core_fhSetter(core_t* core, float val) { core->fh = val; }
float core_fhGetter(core_t* core) { return core->fh; }

void core_fhMaxSetter(core_t* core, float val) { core->fhMax = val; }
float core_fhMaxGetter(core_t* core) { return core->fhMax; }

void core_walkCtLimSetter(core_t* core, float val) { core->walkCtLim = (int)val; }
float core_walkCtLimGetter(core_t* core) { return (float)core->walkCtLim; }

void core_autoHSetter(core_t* core, float val) { core->autoH = val; }
float core_autoHGetter(core_t* core) { return core->autoH; }

#pragma endregion // !core_t getters/setters

#pragma region keyMode5 +- variable tuning

static void keyCont_keyMode5_charPressed_variableChanged(void** state, char plusMinus, int kn, float initial, float expected, void(*varSetter)(core_t*,float), float (*varGetter)(core_t*)) {
    teststate_t* data = *state;
    float result;

    // arrange
    data->input.keyMode = 5;
    (*varSetter)(&data->core, initial);
    data->input.kn = kn;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, plusMinus);
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    result = (*varGetter)(&data->core);
    assert_float_equal(result, expected, 0.001f);
}

void keyCont_keyMode5_dxiIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 0, 100.0f, 101.0f, &core_dxiSetter, &core_dxiGetter);
}

void keyCont_keyMode5_dxiDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 0, 100.0f, 99.0f, &core_dxiSetter, &core_dxiGetter);
}

void keyCont_keyMode5_dyiIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 1, 100.0f, 101.0f, &core_dyiSetter, &core_dyiGetter);
}

void keyCont_keyMode5_dyiDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 1, 100.0f, 99.0f, &core_dyiSetter, &core_dyiGetter);
}

void keyCont_keyMode5_swMaxIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 2, 100.0f, 101.0f, &core_swMaxSetter, &core_swMaxGetter);
}

void keyCont_keyMode5_swMaxDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 2, 100.0f, 99.0f, &core_swMaxSetter, &core_swMaxGetter);
}

void keyCont_keyMode5_pitchGyrgIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 3, 1.0f, 1.01f, &core_pitchGyrgSetter, &core_pitchGyrgGetter);
}

void keyCont_keyMode5_pitchGyrgDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 3, 1.0f, 0.99f, &core_pitchGyrgSetter, &core_pitchGyrgGetter);
}

void keyCont_keyMode5_rollGyrgIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 4, 1.0f, 1.01f, &core_rollGyrgSetter, &core_rollGyrgGetter);
}

void keyCont_keyMode5_rollGyrgDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 4, 1.0f, 0.99f, &core_rollGyrgSetter, &core_rollGyrgGetter);
}

void keyCont_keyMode5_fhIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 5, 1.0f, 2.0f, &core_fhSetter, &core_fhGetter);
}

void keyCont_keyMode5_fhDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 5, 1.0f, 0.0f, &core_fhSetter, &core_fhGetter);
}

void keyCont_keyMode5_fhMaxIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 6, 1.0f, 2.0f, &core_fhMaxSetter, &core_fhMaxGetter);
}

void keyCont_keyMode5_fhMaxDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 6, 1.0f, 0.0f, &core_fhMaxSetter, &core_fhMaxGetter);
}

void keyCont_keyMode5_walkCtLimIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 7, 1.0f, 2.0f, &core_walkCtLimSetter, &core_walkCtLimGetter);
}

void keyCont_keyMode5_walkCtLimDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 7, 1.0f, 0.0f, &core_walkCtLimSetter, &core_walkCtLimGetter);
}

void keyCont_keyMode5_autoHIncremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '+', 8, 1.0f, 2.0f, &core_autoHSetter, &core_autoHGetter);
}

void keyCont_keyMode5_autoHDecremented(void** state) {
    keyCont_keyMode5_charPressed_variableChanged(state, '-', 8, 1.0f, 0.0f, &core_autoHSetter, &core_autoHGetter);
}

#pragma endregion // !keyMode5 +- variable tuning

// TODO keyMode 2: '0', '1', '2'
// TODO keyMode 3: '0', '1', '2'
// TODO keyMode 4: '0', '1'
// TODO keyMode between 20..60: 'r', 'l', 'b'
// TODO keyMode between 200..800: '+', '-'
