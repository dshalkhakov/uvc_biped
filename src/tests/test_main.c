#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>
#include <kcb5.h>
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
    assert_true(1);
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
        cmocka_unit_test(detAng_pitchAndRollInThreshold_doesNothing),
        cmocka_unit_test(uvcSub_whenFwctIs11_supportLegIsReturnedWithMaxValue_inLRdirection),
        cmocka_unit_test(uvcSub_legLiftHeightLessThanMaxLiftHeight_noRoll_legLengthRestored),
        cmocka_unit_test(uvcSub_legLiftHeightLessThanMaxLiftHeight_withRoll_shockAbsorbedWithlegLength),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
