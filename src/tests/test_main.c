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

// this test is ought to be split because it checks so many things at once
static void main_init_doesFoo(void** state) {
    // arrange
    state_t st;
    core_t core;
    input_t input;

    // timer invocation #1 in first delay()
    will_return(__wrap_timer_init, true);
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    expect_any(__wrap_uart_init, port);
    expect_any(__wrap_uart_init, mode);
    expect_any(__wrap_uart_init, baudrate);
    expect_any(__wrap_uart_init, data);
    expect_any(__wrap_uart_init, parity);
    will_return(__wrap_uart_init, true);

    // i2c_read call #1
    expect_any(__wrap_i2c_read, i2c_address);
    expect_any(__wrap_i2c_read, command);
    expect_any(__wrap_i2c_read, c_size);
    will_return(__wrap_i2c_read, 0xA0);
    will_return(__wrap_i2c_read, 1);
    expect_any(__wrap_i2c_read, r_size);

    // timer invocation #2 in second delay()
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #3 in third delay()
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // i2c_read call #2
    expect_any(__wrap_i2c_read, i2c_address);
    expect_any(__wrap_i2c_read, command);
    expect_any(__wrap_i2c_read, c_size);
    will_return(__wrap_i2c_read, 0xA0);
    will_return(__wrap_i2c_read, 1);
    expect_any(__wrap_i2c_read, r_size);

    // timer invocation #4 in fourth delay(50)
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #5 in fourth delay(50)
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #6 in fourth delay(50)
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #7 in fourth delay(50)
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #8 in fourth delay(50)
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #9 in fourth delay(50)
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #10 in fourth delay(50)
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);
    will_return(__wrap_timer_read, -1);

    // timer invocation #11
    will_return(__wrap_timer_write, true);
    will_return(__wrap_timer_start, true);

    // act
    int ret = main_init(&st, &core, &input);

    // assert
    assert_int_equal(ret, 7500);
    assert_int_equal(st.K0W[0], 0);
    assert_int_equal(st.K0W[1], 0);
    assert_int_equal(st.K1W[0], 0);
    assert_int_equal(st.K1W[1], 0);
    assert_int_equal(st.K2W[0], 0);
    assert_int_equal(st.K2W[1], 0);
    assert_int_equal(st.HW[0], 0);
    assert_int_equal(st.HW[1], 0);
    assert_int_equal(st.A0W[0], 0);
    assert_int_equal(st.A0W[1], 0);
    assert_int_equal(st.A1W[0], 0);
    assert_int_equal(st.A1W[1], 0);
    assert_int_equal(st.U0W[0], -2765);
    assert_int_equal(st.U0W[1], -1820);
    assert_int_equal(st.U1W[0], 2610);
    assert_int_equal(st.U1W[1], 2650);
    assert_int_equal(st.U2W[0], 0);
    assert_int_equal(st.U2W[1], 0);
    assert_int_equal(st.EW[0], 2700);
    assert_int_equal(st.EW[1], 2650);
    assert_int_equal(st.HEADW, 0);
    assert_int_equal(st.WESTW, 0);
}

// not sure if this invariant name is correct tho
void state_init_setsBothShouldersBackwards(void** state) {
    // arrange
    state_t st;

    // act
    state_init(&st);

    // assert
    assert_int_equal(-5400, st.U0W[0]);
    assert_int_equal(-5400, st.U0W[1]);
}

void state_init_zeroesServos(void** state) {
    // arrange
    state_t st;

    // act
    state_init(&st);

    // assert
    assert_int_equal(st.K0W[0], 0);			// For writing right hip joint anteroposterior
    assert_int_equal(st.K1W[0], 0);			// For hip joint lateral right writing
    assert_int_equal(st.K2W[0], 0);			// For hip joint lateral right writing
    assert_int_equal(st.HW[0], 0);			// For knee joint right writing
    assert_int_equal(st.A0W[0], 0);			// For ankle upper and lower direction right writing
    assert_int_equal(st.A1W[0], 0);			// For ankle lateral right writing 
    assert_int_equal(st.U1W[0], 0);			// For shoulder horizontal backward right writing
    assert_int_equal(st.U2W[0], 0);			// For shoulder yaw direction right writing
    assert_int_equal(st.EW[0], 0);			// For writing on the right elbow
    assert_int_equal(st.WESTW, 0);			// For writing waist rotation

    assert_int_equal(st.K0W[1], 0);			// For writing left hip joint anteroposterior direction
    assert_int_equal(st.K1W[1], 0);			// For hip joint lateral left writing
    assert_int_equal(st.K2W[1], 0);			// For hip joint lateral left writing
    assert_int_equal(st.HW[1], 0);			// For knee joint left writing
    assert_int_equal(st.A0W[1], 0);			// For writing in the upper and lower direction of the left ankle
    assert_int_equal(st.A1W[1], 0);			// For ankle lateral left writing
    assert_int_equal(st.U1W[1], 0);			// Shoulder horizontal backward direction left writing
    assert_int_equal(st.U2W[1], 0);			// For shoulder yaw direction left writing
    assert_int_equal(st.EW[1], 0);			// For elbow left writing
    assert_int_equal(st.HEADW, 0);			// For head rotation writing
}

void core_init_zeroesValue(void** state) {
    // arrange
    core_t core;

    // act
    core_init(&core);

    // assert
    assert_int_equal(core.LEDct, 0);

    assert_int_equal(core.tBak, 0);
    assert_int_equal(core.pitchi, 0);
    assert_int_equal(core.tNow, 0);

    assert_int_equal(core.p_ofs, 0);
    assert_int_equal(core.r_ofs, 0);
    assert_int_equal(core.ir, 0);
    assert_int_equal(core.ip, 0);
    assert_int_equal(core.irb, 0);
    assert_int_equal(core.ipb, 0);
}

void core_init_setsInitialValues(void** state) {
    // arrange
    core_t core;

    // act
    core_init(&core);

    // assert
    assert_int_equal(core.motCt, 100);
    assert_int_equal(core.cycle, 10000);
    assert_int_equal(core.mode, 710);
    assert_float_equal(core.pitch_gyrg, 0.08f, 0.001f);
    assert_float_equal(core.roll_gyrg, 0.1, 0.001f);

    assert_float_equal(core.swMax, 25, 0.1f);
    assert_float_equal(core.fhMax, 35, 0.1f);
    assert_int_equal(core.walkCtLim, 3);
}

// TODO uvcSub
// TODO uvcSub2
// TODO uvc
// TODO footUp
// TODO swCont
// TODO armCont

// footCont
void footCont_initialPosture_hipPitchChanged(void** state) {
    // arrange
    core_t core;
    state_t st;

    core_init(&core);
    state_init(&st);

    // act
    footCont(&core, &st, vec2_zero, 185, 0);

    // assert
    assert_int_equal(st.K0W[0], 50);
    assert_int_equal(st.K0W[1], 0);
}

void footCont_initialPosture_kneeJointMoved(void** state) {
    // arrange
    core_t core;
    state_t st;

    core_init(&core);
    state_init(&st);

    // act
    footCont(&core, &st, vec2_zero, 185, 0);

    // assert
    assert_int_equal(st.HW[0], 100);
    assert_int_equal(st.HW[1], 0);
}

void footCont_initialPosture_ankleMoved(void** state) {
    // arrange
    core_t core;
    state_t st;

    core_init(&core);
    state_init(&st);

    // act
    footCont(&core, &st, vec2_zero, 185, 0);

    // assert
    assert_int_equal(st.A0W[0], 50);
    assert_int_equal(st.A0W[1], 0);
}

void feetCont1_doesFoo(void** state) {
    core_t core;
    state_t st;

    core_init(&core);
    state_init(&st);

    feetCont1(&core, &st, vec2_zero, vec2_zero, 0);
}

// TODO feetCont2

void counterCont_cycle_fwctProgresses(void** state) {
    // arrange
    core_t core;

    core_init(&core);
    core.jikuasi = 1;
    core.fwctEnd = 18;
    core.fwct   = 0;
    core.fwctUp = 1;

    // act
    counterCont(&core);

    // assert
    assert_int_equal(1, core.jikuasi);
    assert_float_equal(core.fwct, 1.0f, 0.000001);
}

void counterCont_cycle_fwctLimitedByFwctMax(void** state) {
    // arrange
    core_t core;

    core_init(&core);
    core.jikuasi = 1;
    core.fwctEnd = 18;
    core.fwct = 17;
    core.fwctUp = 2;

    // act
    counterCont(&core);

    // assert
    assert_int_equal(1, core.jikuasi);
    assert_float_equal(core.fwct, core.fwctEnd, 0.000001);
}

void counterCont_endOfCycle_groundedFootChanged(void** state) {
    // arrange
    core_t core;

    core_init(&core);
    core.jikuasi = 0;
    core.fwctEnd = 18;
    core.fwct   = 18;

    // act
    counterCont(&core);

    // assert
    assert_int_equal(1, core.jikuasi);
}

void counterCont_endOfCycle_fhReset(void** state) {
    // arrange
    core_t core;

    core_init(&core);
    core.jikuasi = 0;
    core.fwctEnd = 18;
    core.fwct = 18;

    // act
    counterCont(&core);

    // assert
    assert_float_equal(core.fh, 0.0f, 0.001);
}

void counterCont_endOfCycle_dNi_dNisSwapped(void** state) {
    // arrange
    core_t core;

    core_init(&core);
    core.fwctEnd = 18;
    core.fwct = 18;
    core.dxis = 0;
    core.dyis = 0;
    core.dxi = DEGREES2RADIANS(5);
    core.dyi = DEGREES2RADIANS(3);

    // act
    counterCont(&core);

    // assert
    assert_float_equal(core.dxi, 0.0f, 0.00001);
    assert_float_equal(core.dyi, 0.0f, 0.00001);
    assert_float_equal(core.dxis, DEGREES2RADIANS(5), 0.00001);
    assert_float_equal(core.dyis, DEGREES2RADIANS(3), 0.00001);
}

void counterCont_endOfCycle_dNi_dNibSwapped(void** state) {
    // arrange
    core_t core;

    core_init(&core);
    core.fwctEnd = 18;
    core.fwct = 18;
    core.dxis = 0;
    core.dyis = 0;
    core.dxib = 0;
    core.dyib = 0;
    core.dxi = DEGREES2RADIANS(5);
    core.dyi = DEGREES2RADIANS(2);

    // act
    counterCont(&core);

    // assert
    assert_float_equal(core.dxi, 0.0f, 0.000001);
    assert_float_equal(core.dyi, 0.0f, 0.000001);
    assert_float_equal(core.dxib, DEGREES2RADIANS(5), 0.00001);
    assert_float_equal(core.dyib, DEGREES2RADIANS(2), 0.00001);
}

// TODO walk

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
        cmocka_unit_test(main_init_doesFoo),
        cmocka_unit_test(state_init_setsBothShouldersBackwards),
        cmocka_unit_test(state_init_zeroesServos),
        cmocka_unit_test(core_init_setsInitialValues),
        cmocka_unit_test(core_init_zeroesValue),
        cmocka_unit_test(footCont_initialPosture_hipPitchChanged),
        cmocka_unit_test(footCont_initialPosture_kneeJointMoved),
        cmocka_unit_test(footCont_initialPosture_ankleMoved),
        cmocka_unit_test(counterCont_cycle_fwctProgresses),
        cmocka_unit_test(counterCont_cycle_fwctLimitedByFwctMax),
        cmocka_unit_test(counterCont_endOfCycle_groundedFootChanged),
        cmocka_unit_test(counterCont_endOfCycle_fhReset),
        cmocka_unit_test(counterCont_endOfCycle_dNi_dNisSwapped),
        cmocka_unit_test(counterCont_endOfCycle_dNi_dNibSwapped),
    };

    int keyContRc = test_keyCont();
    if (keyContRc)
    {
        return keyContRc;
    }
    int angleRepresentationsRc = test_angleRepresentations();
    if (angleRepresentationsRc)
    {
        return angleRepresentationsRc;
    }

    return cmocka_run_group_tests_name("general", tests, NULL, NULL);
}
