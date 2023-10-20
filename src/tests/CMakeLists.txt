project(tests)

include(CTest)

#find_package(check CONFIG REQUIRED)
find_package(cmocka CONFIG REQUIRED)

#add_executable(tests test_main.c)
add_executable(tests test_main2.c)

#target_link_libraries(tests PRIVATE biped_controller) # linking directly won't work, don't even try
#target_link_libraries(tests PRIVATE Check::check Check::checkShared)
target_link_libraries(tests PRIVATE cmocka::cmocka)
