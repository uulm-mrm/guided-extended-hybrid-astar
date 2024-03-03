set(LIBRARY_NAME util_lib)

find_package (pybind11 REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# we default to Release build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(${LIBRARY_NAME}
        util1.cpp
        util2.cpp
        transforms.cpp
        )

add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

target_link_libraries(${LIBRARY_NAME} PUBLIC
        stdc++fs
        pybind11::module
        pybind11::embed
        )

target_include_directories(${LIBRARY_NAME} SYSTEM PUBLIC)
target_include_directories(${LIBRARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
        )

if (${COVERAGE})
    message(WARNING "COVERAGE IS ON")
    target_link_libraries(${LIBRARY_NAME}
            PRIVATE
            -fprofile-arcs -ftest-coverage
    )
    target_compile_options(${LIBRARY_NAME} PRIVATE --coverage -g -O0)
endif()

target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_20)