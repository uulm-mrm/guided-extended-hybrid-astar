find_package (pybind11)
find_package (OpenCV REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# we default to Debug build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(pybind11_FOUND)
    set(PY_TARGET_NAME "_gridmap_sim_lib_api")

    # Collect files of this module
    set(SOURCE_FILES
            wrapper_gridmap_sim.cpp
            gridmap_sim.cpp
            )

    set(HEADER_FILES
            ../../include/gridmap_sim_lib/gridmap_sim.hpp
            )

    # Builds the python bindings module.
    pybind11_add_module(${PY_TARGET_NAME} MODULE ${SOURCE_FILES})

    target_link_libraries(${PY_TARGET_NAME} PRIVATE
            stdc++fs
            yaml-cpp
            pybind11::module
            pybind11::embed
            ${OpenCV_LIBS}
            )

    target_include_directories(${PY_TARGET_NAME} SYSTEM PUBLIC)
    target_include_directories(${PY_TARGET_NAME} PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
            )

    if (${COVERAGE})
        message(WARNING "COVERAGE IS ON")
        target_link_libraries(${PY_TARGET_NAME}
                PRIVATE
                -fprofile-arcs -ftest-coverage
        )
        target_compile_options(${PY_TARGET_NAME} PRIVATE --coverage -g -O0)
    endif()

    target_compile_features(${PY_TARGET_NAME} PUBLIC cxx_std_20)

    # Install in python directory
    install(TARGETS ${PY_TARGET_NAME}
            LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/gridmap_sim_lib)

    # Python module
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/python/gridmap_sim_lib/
            DESTINATION ${PYTHON_INSTALL_DIR}/gridmap_sim_lib)
endif()