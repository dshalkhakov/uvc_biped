#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <cmocka.h>
#include <kcb5.h>
#include <uart.h>
#include "../main.h"

#define SVANGLE_EPSILON     (0.025f)

static void ANGLE2SVANGLE_when180_correct(void** state) {
    float angle = 180.0f;
    int16_t converted = ANGLE2SVANGLE(angle);

    assert_int_equal(converted, SVANGLE_180);
}

static void ANGLE2SVANGLE_when60_correct(void** state) {
    float angle = 60.0f;
    int16_t converted = ANGLE2SVANGLE(angle);

    assert_int_equal(converted, 1777);
}

static void SVANGLE2ANGLE_when180_correct(void** state) {
    int16_t angle = SVANGLE_180;
    float converted = SVANGLE2ANGLE(angle);

    assert_float_equal(converted, 180.0f, SVANGLE_EPSILON);
}

static void SVANGLE2ANGLE_when60_correct(void** state) {
    int16_t angle = 1777;
    float converted = SVANGLE2ANGLE(angle);

    assert_float_equal(converted, 60.0f, SVANGLE_EPSILON);
}

int test_angleRepresentations() {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(ANGLE2SVANGLE_when180_correct),
        cmocka_unit_test(ANGLE2SVANGLE_when60_correct),
        cmocka_unit_test(SVANGLE2ANGLE_when180_correct),
        cmocka_unit_test(SVANGLE2ANGLE_when60_correct),
    };

    return cmocka_run_group_tests_name("angleRepresentations", tests, NULL, NULL);
}
