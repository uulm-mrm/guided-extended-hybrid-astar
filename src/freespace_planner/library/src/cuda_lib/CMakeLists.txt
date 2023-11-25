set(LIBRARY_NAME cuda_lib)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

enable_language(CUDA)
include(CheckLanguage)
check_language(CUDA)

find_package(CUDA REQUIRED)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -O3)

# we default to Release build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(${LIBRARY_NAME}
        max_pool.cu
)

add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})


target_link_libraries(${LIBRARY_NAME} PUBLIC
        stdc++fs
        ${CUDA_LIBRARIES}
        cudnn
        )


target_include_directories(${LIBRARY_NAME} SYSTEM PUBLIC)
target_include_directories(${LIBRARY_NAME} PUBLIC
        ${CUDA_INCLUDE_DIRS}
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
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