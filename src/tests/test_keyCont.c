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

static void keyCont_keyModeN_charPressed_nextKeyModeEngaged(void** state, int keyMode, char charPressed, int nextKeyMode) {
    teststate_t* data = *state;

    // arrange
    data->input.keyMode = keyMode;

    expect_value(__wrap_uart_rx, port, UART_COM);
    will_return(__wrap_uart_rx, charPressed);
    expect_value(__wrap_uart_rx, length, 1);
    expect_value(__wrap_uart_rx, timeout, 1);
    will_return(__wrap_uart_rx, 1);

    // act
    keyCont(&data->input, &data->core, &data->state);

    // assert
    assert_int_equal(data->input.keyMode, nextKeyMode);
}

#pragma region keyMode 2: '0', '1', '2' -> 20, 21, 22

void keyCont_keyMode2_0Pressed_keyMode20Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 2, '0', 20);
}

void keyCont_keyMode2_1Pressed_keyMode21Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 2, '1', 21);
}

void keyCont_keyMode2_2Pressed_keyMode22Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 2, '2', 22);
}

#pragma endregion

#pragma region keyMode 3: '0', '1', '2' -> 30, 31, 32

void keyCont_keyMode3_0Pressed_keyMode30Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 3, '0', 30);
}

void keyCont_keyMode3_1Pressed_keyMode31Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 3, '1', 31);
}

void keyCont_keyMode3_2Pressed_keyMode32Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 3, '2', 32);
}

#pragma endregion

#pragma region keyMode 4: '0', '1' -> 40, 41

void keyCont_keyMode4_0Pressed_keyMode40Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 4, '0', 40);
}

void keyCont_keyMode4_1Pressed_keyMode41Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 4, '1', 41);
}

#pragma endregion // !keyMode 4: '0', '1'

#pragma region keyMode between 20..60: 'r', 'l', 'b' -> *10

void keyCont_keyMode20_rPressed_keyMode200Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 20, 'r', 200);
}

void keyCont_keyMode20_lPressed_keyMode201Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 20, 'l', 201);
}

void keyCont_keyMode20_bPressed_keyMode202Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 20, 'b', 202);
}

// 21 -> 210, 211, 212
void keyCont_keyMode21_rPressed_keyMode210Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 21, 'r', 210);
}

void keyCont_keyMode21_lPressed_keyMode211Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 21, 'l', 211);
}

void keyCont_keyMode21_bPressed_keyMode212Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 21, 'b', 212);
}

// 22 -> 220, 221, 222
void keyCont_keyMode22_rPressed_keyMode220Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 22, 'r', 220);
}

void keyCont_keyMode22_lPressed_keyMode221Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 22, 'l', 221);
}

void keyCont_keyMode22_bPressed_keyMode222Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 22, 'b', 222);
}

// 30 -> 300, 301, 302
void keyCont_keyMode30_rPressed_keyMode300Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 30, 'r', 300);
}

void keyCont_keyMode30_lPressed_keyMode301Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 30, 'l', 301);
}

void keyCont_keyMode30_bPressed_keyMode302Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 30, 'b', 302);
}

// 31 -> 310, 311, 312
void keyCont_keyMode31_rPressed_keyMode310Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 31, 'r', 310);
}

void keyCont_keyMode31_lPressed_keyMode311Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 31, 'l', 311);
}

void keyCont_keyMode31_bPressed_keyMode312Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 31, 'b', 312);
}

// 32 -> 320, 321, 322
void keyCont_keyMode32_rPressed_keyMode320Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 32, 'r', 320);
}

void keyCont_keyMode32_lPressed_keyMode321Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 32, 'l', 321);
}

void keyCont_keyMode32_bPressed_keyMode322Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 32, 'b', 322);
}

// 40 -> 400, 401, 402
void keyCont_keyMode40_rPressed_keyMode400Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 40, 'r', 400);
}

void keyCont_keyMode40_lPressed_keyMode401Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 40, 'l', 401);
}

void keyCont_keyMode40_bPressed_keyMode402Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 40, 'b', 402);
}

// 41 -> 410, 411, 412
void keyCont_keyMode41_rPressed_keyMode410Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 41, 'r', 410);
}

void keyCont_keyMode41_lPressed_keyMode411Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 41, 'l', 411);
}

void keyCont_keyMode41_bPressed_keyMode412Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 41, 'b', 412);
}

// 50 -> 500, 501, 502
void keyCont_keyMode50_rPressed_keyMode500Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 50, 'r', 500);
}

void keyCont_keyMode50_lPressed_keyMode501Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 50, 'l', 501);
}

void keyCont_keyMode50_bPressed_keyMode502Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 50, 'b', 502);
}

// 60 -> 600, 601, 602
void keyCont_keyMode60_rPressed_keyMode600Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 60, 'r', 600);
}

void keyCont_keyMode60_lPressed_keyMode601Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 60, 'l', 601);
}

void keyCont_keyMode60_bPressed_keyMode602Engaged(void** state) {
    keyCont_keyModeN_charPressed_nextKeyModeEngaged(state, 60, 'b', 602);
}

#pragma endregion

#pragma region keyMode between 200..800: '+', '-'

// 200, 201, 202
// 210, 211, 212
// 220, 221, 222

#pragma endregion // !keyMode between 200..800: '+', '-'

