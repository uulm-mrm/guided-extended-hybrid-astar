cmake_minimum_required(VERSION 3.18)
project(freespace_planner_lib)

message("SANDBOX_PATH ${SANDBOX_PATH}")
set(INSTALL_PATH ${SANDBOX_PATH}/install/${PROJECT_NAME})
set(PYTHON_INST_PATH ${INSTALL_PATH}/lib/python3.10/site-packages)

message("INSTALL_PATH ${INSTALL_PATH}")
message("PYTHON_INST_PATH ${PYTHON_INST_PATH}")

# overwrite variable normally set by colcon
set(PYTHON_INSTALL_DIR ${PYTHON_INST_PATH})
set(LIB_INSTALL_DIR ${INSTALL_PATH}/lib)
set(BIN_INSTALL_DIR ${INSTALL_PATH}/bin)
set(INCLUDE_INSTALL_DIR ${INSTALL_PATH}/include)

message("SB root is at ${SANDBOX_PATH}")
message("Installing to ${INSTALL_PATH}")
message("Py installs to ${PYTHON_INST_PATH}")

# Only cpp stuff
add_subdirectory(src/cartographing_lib)

add_subdirectory(src/gridmap_sim_lib)

add_subdirectory(src/util_lib)

add_subdirectory(src/cuda_lib)

add_subdirectory(src/deps_lib)

add_subdirectory(src/collision_checker_lib)

add_subdirectory(src/path_planner_lib)

# Mixed python and cpp
add_subdirectory(src/hybridastar_planning_lib)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
        _hybridastar_planning_lib_api
)

# Install files for all targets
set(LIBRARY_TARGETS collision_checker_lib util_lib deps_lib)
#set(TEST_EXECUTABLES util_lib_test)
install(TARGETS ${LIBRARY_TARGETS} DESTINATION ${LIB_INSTALL_DIR})

#message("Copy packages to env")
#message("from ${PYTHON_INST_PATH}/")
#message("to ${SANDBOX_PATH}/env/lib/python3.10/site-packages/")

## install to local venv
#INSTALL(CODE "execute_process( \
#    COMMAND cp -r ${PYTHON_INST_PATH}/ ${SANDBOX_PATH}/env/lib/python3.10/site-packages/\
#    )"
#)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
