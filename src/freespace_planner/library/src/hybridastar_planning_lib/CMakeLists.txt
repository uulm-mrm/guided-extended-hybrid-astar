find_package (pybind11)
find_package (OpenCV REQUIRED)
find_package(GLEW REQUIRED)

#find_package(aduulm_logger_lib REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# we default to Release build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(pybind11_FOUND)
    set(PY_TARGET_NAME "_hybridastar_planning_lib_api")

    # must be the same as folder name
    set(MODULE_NAME "hybridastar_planning_lib")

    # Collect files of this module
    set(SOURCE_FILES
#            logger_setup.cpp
            smoother.cpp
            wrapper_hybrid_a_star_lib.cpp
            hybrid_a_star_lib.cpp
            a_star.cpp
    )

    # Builds the python bindings module.
    pybind11_add_module(${PY_TARGET_NAME} MODULE ${SOURCE_FILES})

    target_link_libraries(${PY_TARGET_NAME} PRIVATE
            stdc++fs
            yaml-cpp
            cartographing_lib
            util_lib
            deps_lib
            cuda_lib
            collision_checker_lib
            pybind11::module
            pybind11::embed
            ${OpenCV_LIBS}
    )

    # Specify libraries to link
#    link_aduulm_package_targets(TARGET ${PY_TARGET_NAME}
#            ACCESS PUBLIC
#            PACKAGE_TARGETS
#            aduulm_logger_lib::aduulm_logger_lib
#    )


    target_include_directories(${PY_TARGET_NAME} SYSTEM PUBLIC)
    target_include_directories(${PY_TARGET_NAME} PUBLIC
            ${OpenCV_INCLUDE_DIRS}
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

    target_compile_features(${PY_TARGET_NAME} PRIVATE cxx_std_20)

    # Install cpp binary in python directory
    install(TARGETS ${PY_TARGET_NAME}
            LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${MODULE_NAME})

    # Symlink Python module
    set(PYTHON_MODULE_INST_DIR ${PYTHON_INST_PATH}/${MODULE_NAME})
    INSTALL(CODE "execute_process( \
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/python/${MODULE_NAME}/__init__.py \
    ${PYTHON_MODULE_INST_DIR}/__init__.py \
    )"
    )

    else()
    message(WARNING "Pybind11 not found, skip building python bindings")
endif()