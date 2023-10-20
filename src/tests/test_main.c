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

static void null_test_success(void** state) {
    (void)state; /* unused */

    assert_int_equal(2, 2);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(movSv_whenMotCtLessThan1_stopsMoving),
        cmocka_unit_test(movSv_whenMotCtMoreThan1_moves),
        cmocka_unit_test(null_test_success),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
