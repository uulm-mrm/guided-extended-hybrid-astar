set(LIBRARY_NAME deps_lib)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package (OpenCV REQUIRED)

# we default to Release build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

## FITPACK FORTRAN code
enable_language(Fortran)
file(GLOB fitpack_SRC CONFIGURE_DEPENDS
        "fitpack/*.f"
)
add_library(fitpack STATIC
        ${fitpack_SRC}
)

enable_language(CXX)

# C++ interface
include(FortranCInterface)
FortranCInterface_HEADER(FCMangle.h
        MACRO_NAMESPACE "FC_"
        SYMBOL_NAMESPACE ""
        SYMBOLS curfit splev)
FortranCInterface_VERIFY(CXX)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
# suppress fortran warnings of scipy
set_target_properties(fitpack PROPERTIES COMPILE_FLAGS "-w")


add_library(${LIBRARY_NAME}
        reeds_shepp.cpp
        VoronoiDiagramGenerator.cpp
        BSpline1D.cpp
)
# suppress warnings of external dependency
set_target_properties(${LIBRARY_NAME} PROPERTIES COMPILE_FLAGS "-w")

add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})


target_link_libraries(${LIBRARY_NAME} PUBLIC
        stdc++fs
        util_lib
        fitpack
        ${OpenCV_LIBS}
        )

target_include_directories(${LIBRARY_NAME} SYSTEM PUBLIC)
target_include_directories(${LIBRARY_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIRS}
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