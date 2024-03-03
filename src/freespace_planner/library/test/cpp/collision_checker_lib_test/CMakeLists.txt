# find and enable GTest
find_package(GTest REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)

if(NOT ${GTEST_FOUND})
    message(WARNING "GTest not found, skipping tests for imu_processing_lib")
    return()
endif()
enable_testing()


# executable
SET(TEST_NAME collision_checker_lib_test)

add_executable(${TEST_NAME}
        test_collision_checking.cpp
)

link_aduulm_package_targets(TARGET ${TEST_NAME}
        ACCESS PUBLIC
        PACKAGE_TARGETS
)

target_link_libraries(${TEST_NAME}
        PRIVATE
        collision_checker_lib
        GTest::gtest_main
        tbb
)

target_compile_features(${TEST_NAME}
        PRIVATE
        cxx_std_20
)


SET(LIBRARY2TEST collision_checker_lib)
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
