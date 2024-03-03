cmake_minimum_required(VERSION 3.18)
project(freespace_planner_lib)

find_package(aduulm_cmake_tools REQUIRED)

get_version_string_from_package_xml(.)
show_build_info("library")
setup_git_hooks()

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

# get sandbox path for symlink install of submodules
cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH PARENT_DIR)
cmake_path(GET PARENT_DIR PARENT_PATH LIBRARY_PATH)
cmake_path(GET LIBRARY_PATH PARENT_PATH SANDBOX_PATH)

set(INSTALL_PATH ${SANDBOX_PATH}/colcon_build/install/${PROJECT_NAME})
set(PYTHON_INST_PATH ${INSTALL_PATH}/${PYTHON_INSTALL_DIR})

## Only cpp stuff
add_subdirectory(src/cartographing_lib)

add_subdirectory(src/gridmap_sim_lib)

add_subdirectory(src/util_lib)

add_subdirectory(src/cuda_lib)

add_subdirectory(src/deps_lib)

add_subdirectory(src/collision_checker_lib)

add_subdirectory(src/path_planner_lib)

## Mixed python and cpp
add_subdirectory(src/hybridastar_planning_lib)


# Test
add_subdirectory(test/cpp/util_lib_test)
add_subdirectory(test/cpp/collision_checker_lib_test)

set(SUB_TARGETS _hybridastar_planning_lib_api collision_checker_lib util_lib deps_lib cuda_lib _gridmap_sim_lib_api cartographing_lib
                fitpack)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} INTERFACE
        _hybridastar_planning_lib_api
        )

# Install config (symlink)
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

# Install data (symlink)
install(DIRECTORY
        data
        DESTINATION share/${PROJECT_NAME}
)

# Create package version file
set(PACKAGE_LIBRARY_VERSION "${package_version}")
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        VERSION "${PACKAGE_LIBRARY_VERSION}"
        COMPATIBILITY AnyNewerVersion
)

# Install test files
set(TEST_EXECUTABLES util_lib_test collision_checker_lib_test)

# Install all files
install(TARGETS ${PROJECT_NAME} ${TEST_EXECUTABLES} ${SUB_TARGETS}
        EXPORT ${PROJECT_NAME}Targets # store targets in variable
        INCLUDES DESTINATION ${INCLUDE_INSTALL_DIR}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT Runtime
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT Development
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT Runtime
        PUBLIC_HEADER DESTINATION ${INCLUDE_INSTALL_DIR} COMPONENT Development
        BUNDLE DESTINATION ${BIN_INSTALL_DIR} COMPONENT Runtime
        )

# Create package config file (to enable find_package())
configure_package_config_file(
        "${PROJECT_SOURCE_DIR}/config.cmake.in" # this file defines the include dirs and libs and may need to be adapted for your package/library
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
        PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR BIN_INSTALL_DIR
)

# Create targets file with all targets (enables to link against single targets)
install(EXPORT ${PROJECT_NAME}Targets
        DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
        NAMESPACE ${PROJECT_NAME}::
)

# install CMake config and version file
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME})

# install header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${INCLUDE_INSTALL_DIR})


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
