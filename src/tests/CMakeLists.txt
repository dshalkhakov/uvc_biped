project(tests)

find_package(cmocka CONFIG REQUIRED)

add_executable(tests "test_main.c" "../main.h" "timer_mock.c" "uart_mock.c" "sys_stubs.c" "test_keyCont.c" "test_keyCont.h" "i2c_mock.c" "test_angleRepresentations.c" "test_angleRepresentations.h")

target_link_libraries(tests PRIVATE cmocka::cmocka biped_lib)
