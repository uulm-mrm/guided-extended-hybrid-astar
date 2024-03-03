# find and enable GTest
find_package(GTest REQUIRED)
if(NOT ${GTEST_FOUND})
    message(WARNING "GTest not found, skipping tests for imu_processing_lib")
    return()
endif()
enable_testing()


# executable
SET(TEST_NAME util_lib_test)

add_executable(${TEST_NAME}
        test_util.cpp
)

link_aduulm_package_targets(TARGET ${TEST_NAME}
        ACCESS PUBLIC
        PACKAGE_TARGETS
)

target_link_libraries(${TEST_NAME}
        PRIVATE
        util_lib
        GTest::gtest_main
)

target_compile_features(${TEST_NAME}
        PRIVATE
        cxx_std_20
)


SET(LIBRARY2TEST util_lib)
# coverage
if(${COVERAGE})
    message(STATUS "Building unit tests for coverage analysis")
    cmake_policy(SET CMP0079 NEW)  # enable access to library target from subdirectory
    target_link_libraries(${LIBRARY2TEST} PRIVATE gcov)
    target_compile_options(${LIBRARY2TEST} PRIVATE --coverage -g -O0)
    target_link_libraries(${TEST_NAME} PRIVATE gcov)
    target_compile_options(${TEST_NAME} PRIVATE --coverage -g -O0)
endif()


# add tests to GTest
gtest_discover_tests(${TEST_NAME})
