#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>
#include <kcb5.h>
#include <uart.h>
#include "../main.h"

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

extern int keyCont_testSetup(void** state);
extern int keyCont_testTeardown(void** state);
extern void keyCont_nullString_ignored(void** state);
extern void keyCont_spacePressed_keyModeReset(void** state);
extern void keyCont_basicKeyMode_rPressed_coreModeIsReset(void** state);
extern void keyCont_basicKeyMode_gPressed_started(void** state);
extern void keyCont_basicKeyMode_tPressed_coreMode790Engaged(void** state);
extern void keyCont_basicKeyMode_yPressed_coreMode791Engaged(void** state);
extern void keyCont_basicKeyMode_pPressed_keyMode5Engaged(void** state);

extern void keyCont_keyMode5_digit0Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit1Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit2Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit3Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit4Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit5Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit6Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit7Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit8Pressed_knSet(void** state);
extern void keyCont_keyMode5_digit9Pressed_knSet(void** state);
extern void keyCont_keyMode5_dxiIncremented(void** state);
extern void keyCont_keyMode5_dxiDecremented(void** state);
extern void keyCont_keyMode5_dyiIncremented(void** state);
extern void keyCont_keyMode5_dyiDecremented(void** state);
extern void keyCont_keyMode5_swMaxIncremented(void** state);// swMax
extern void keyCont_keyMode5_swMaxDecremented(void** state);// swMax
extern void keyCont_keyMode5_pitchGyrgIncremented(void** state);// pitch_gyrg
extern void keyCont_keyMode5_pitchGyrgDecremented(void** state);// pitch_gyrg
extern void keyCont_keyMode5_rollGyrgIncremented(void** state);// roll_gyrg
extern void keyCont_keyMode5_rollGyrgDecremented(void** state);// roll_gyrg
extern void keyCont_keyMode5_fhIncremented(void** state);// fh
extern void keyCont_keyMode5_fhDecremented(void** state);// fh
extern void keyCont_keyMode5_fhMaxIncremented(void** state);// fhMax
extern void keyCont_keyMode5_fhMaxDecremented(void** state);// fhMax
extern void keyCont_keyMode5_walkCtLimIncremented(void** state);// walkCtLim
extern void keyCont_keyMode5_walkCtLimDecremented(void** state);// walkCtLim
extern void keyCont_keyMode5_autoHIncremented(void** state);// autoH
extern void keyCont_keyMode5_autoHDecremented(void** state);// autoH

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
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
