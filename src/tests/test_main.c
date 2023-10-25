#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>
#include <kcb5.h>
#include <uart.h>
#include "../main.h"
#include "test_keyCont.h"

static void movSv_whenMotCtLessThan1_stopsMoving(void** state) {
    // arrange
    core_t core;
    
    core.motCt = 0;
    short src = 1;
    int dst = 10;

    // act
    movSv(&core, &src, dst);

    // assert
    assert_int_equal(src, dst);
}

static void movSv_whenMotCtMoreThan1_moves(void** state) {
    // arrange
    core_t core;

    core.motCt = 20;
    short src = 1;
    int dst = 100;

    // act
    movSv(&core, &src, dst);

    // assert
    assert_int_equal(src, 5);
}

static void angAdj_foo(void** state) {
    // arrange
    core_t core;

    core.pitchs = TO_HWANGLE(10);
    core.rolls  = TO_HWANGLE(0);
    core.ip     = 10;
    core.ipa    = 10;
    core.ira    = 10;
    core.ipb    = 10;
    core.irb    = 10;

    // act
    angAdj(&core);

    // assert
    assert_int_equal(core.ip, 0);
    assert_int_equal(core.ipa, 0);
    assert_int_equal(core.ira, 0);
    assert_int_equal(core.ipb, TO_HWANGLE(10));
    assert_int_equal(core.irb, TO_HWANGLE(0));
}

static void angAdj_bar(void** state) {
    // arrange
    core_t core;
    int16_t angle = TO_HWANGLE(1);

    core.pitchs = angle;
    core.rolls = angle;
    core.ip = 10;
    core.ipa = 10;
    core.ira = 10;
    core.ipb = angle + 1;
    core.irb = angle - 1;

    // act
    angAdj(&core);

    // assert
    assert_int_equal(core.ip, 11);
    assert_int_equal(core.ipa, 10 + angle);
    assert_int_equal(core.ira, 10 + angle);
    assert_int_equal(core.ipb, angle);
    assert_int_equal(core.irb, angle);
}

static void detAng_pitchAndRollInThreshold_doesNothing(void** state) {
    core_t core;

    // arrange
    core.pitch = 0.1f;
    core.roll = 0.1f;

    // act
    detAng(&core);

    // assert. we know everything went well because detAng goes to while(1) loop otherwise
    assert_true(1);
}

// TODO detAng pitchAndRollExceedThreshold

static void uvcSub_whenFwctIs11_supportLegIsReturnedWithMaxValue_inLRdirection(void** state) {
    core_t core;

    // arrange
    core.fwct = 11;
    core.fwctUp = 1;
    core.fwctEnd = 18;
    core.landF = 25;
    core.dxi = 6.0f;
    core.dxis = 6.0f;
    core.dyi = 6.0f;
    core.dyis = 6.0f;
    core.autoH = 160;

    // act
    uvcSub(&core);

    // assert
    // with above setup, division by zero occurs when calculating dyi. is it on purpose or not?
    // or it never happens in practice?
    assert_float_equal(core.dyi, 70.0f, 0.001f);
}

static void uvcSub_legLiftHeightLessThanMaxLiftHeight_noRoll_legLengthRestored(void** state) {
    core_t core;

    // arrange
    core.jikuasi = 0;
    core.fwct = 18;
    core.fwctUp = 1;
    core.fwctEnd = 18;
    core.landF = 25;
    core.dxi = 6.0f;
    core.dxis = 6.0f;
    core.dyi = 6.0f;
    core.dyis = 6.0f;
    core.autoH = 160;
    core.landB = 0;
    core.roll = 0;
    core.rollt = 0;

    // act
    uvcSub(&core);

    // assert
    assert_float_equal(core.autoH, 161.75f, 0.001f);
}

static void uvcSub_legLiftHeightLessThanMaxLiftHeight_withRoll_shockAbsorbedWithlegLength(void** state) {
    core_t core;

    // arrange
    core.jikuasi = 0;
    core.fwct = 19;
    core.fwctUp = 1;
    core.fwctEnd = 18;
    core.landF = 25;
    core.dxi = 1.0f;
    core.dxis = 1.0f;
    core.dyi = 1.0f;
    core.dyis = 1.0f;
    core.autoH = 160;
    core.landB = 0;
    core.roll = HWANGLE_TORAD(TO_HWANGLE(1.5f));
    core.rollt = 0.25 * core.roll;

    // act
    uvcSub(&core);

    // assert
    assert_float_equal(core.autoH, 140.0f, 0.001f);
}

// TODO same as uvcSub_legLiftHeightLessThanMaxLiftHeight_withRoll_shockAbsorbedWithlegLength, but not capped to 140

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(movSv_whenMotCtLessThan1_stopsMoving),
        cmocka_unit_test(movSv_whenMotCtMoreThan1_moves),
        cmocka_unit_test(angAdj_foo),
        cmocka_unit_test(angAdj_bar),
        cmocka_unit_test(detAng_pitchAndRollInThreshold_doesNothing),
        cmocka_unit_test(uvcSub_whenFwctIs11_supportLegIsReturnedWithMaxValue_inLRdirection),
        cmocka_unit_test(uvcSub_legLiftHeightLessThanMaxLiftHeight_noRoll_legLengthRestored),
        cmocka_unit_test(uvcSub_legLiftHeightLessThanMaxLiftHeight_withRoll_shockAbsorbedWithlegLength),
        cmocka_unit_test_setup_teardown(keyCont_nullString_ignored, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_spacePressed_keyModeReset, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_basicKeyMode_rPressed_coreModeIsReset, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_basicKeyMode_gPressed_started, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_basicKeyMode_tPressed_coreMode790Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_basicKeyMode_yPressed_coreMode791Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_basicKeyMode_pPressed_keyMode5Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit0Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit1Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit2Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit3Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit4Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit5Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit6Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit7Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit8Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_digit9Pressed_knSet, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_dxiIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_dxiDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_dyiIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_dyiDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_swMaxIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_swMaxDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_pitchGyrgIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_pitchGyrgDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_rollGyrgIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_rollGyrgDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_fhIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_fhDecremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_fhMaxIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_fhMaxDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_walkCtLimIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_walkCtLimDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode5_autoHIncremented, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode5_autoHDecremented, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode2_0Pressed_keyMode20Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode2_1Pressed_keyMode21Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode2_2Pressed_keyMode22Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode3_0Pressed_keyMode30Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode3_1Pressed_keyMode31Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode3_2Pressed_keyMode32Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode4_0Pressed_keyMode40Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode4_1Pressed_keyMode41Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode20_rPressed_keyMode200Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode20_lPressed_keyMode201Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode20_bPressed_keyMode202Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode21_rPressed_keyMode210Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode21_lPressed_keyMode211Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode21_bPressed_keyMode212Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode22_rPressed_keyMode220Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode22_lPressed_keyMode221Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode22_bPressed_keyMode222Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode30_rPressed_keyMode300Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode30_lPressed_keyMode301Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode30_bPressed_keyMode302Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode31_rPressed_keyMode310Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode31_lPressed_keyMode311Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode31_bPressed_keyMode312Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode32_rPressed_keyMode320Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode32_lPressed_keyMode321Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode32_bPressed_keyMode322Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode40_rPressed_keyMode400Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode40_lPressed_keyMode401Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode40_bPressed_keyMode402Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode41_rPressed_keyMode410Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode41_lPressed_keyMode411Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode41_bPressed_keyMode412Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode50_rPressed_keyMode500Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode50_lPressed_keyMode501Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode50_bPressed_keyMode502Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode60_rPressed_keyMode600Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode60_lPressed_keyMode601Engaged, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode60_bPressed_keyMode602Engaged, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode200_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode200_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode201_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode201_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode202_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode202_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode210_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode210_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode211_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode211_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode212_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode212_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode220_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode220_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode221_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode221_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode222_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode222_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode300_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode300_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode301_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode301_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode302_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode302_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode310_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode310_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode311_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode311_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode312_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode312_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode320_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode320_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode321_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode321_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode322_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode322_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode400_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode400_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode401_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode401_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode402_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode402_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode410_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode410_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode411_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode411_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode412_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode412_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode500_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode500_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode501_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode501_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode502_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode502_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode600_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode600_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode601_plus_servoForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode601_minus_servoBackward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode602_plus_servosForward, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode602_minus_servosBackward, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode700_plus_headRight, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode700_minus_headLeft, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode701_plus_headRight, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode701_minus_headLeft, keyCont_testSetup, keyCont_testTeardown),

        cmocka_unit_test_setup_teardown(keyCont_keyMode800_plus_waistRight, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode800_minus_waistLeft, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode801_plus_waistAsIs, keyCont_testSetup, keyCont_testTeardown),
        cmocka_unit_test_setup_teardown(keyCont_keyMode801_minus_waistAsIs, keyCont_testSetup, keyCont_testTeardown),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
