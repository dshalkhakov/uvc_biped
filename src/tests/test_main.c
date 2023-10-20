#include <check.h>

/* A test case that does nothing and succeeds. */
START_TEST(test_nothing)
{
    /* unit test code */
    ck_assert_int_eq(0, 0);
}
END_TEST

Suite* default_suite(void)
{
    Suite * s;
    TCase * tc_core;

    s = suite_create("default");

    /* Core test case */
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_nothing);
    suite_add_tcase(s, tc_core);
    
    return s;
}

int main(void)
{
    int number_failed;
    Suite * s;
    SRunner * sr;
    
    s = default_suite();
    sr = srunner_create(s);
    
    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);
    return (number_failed == 0) ? 0 : 1;
}
