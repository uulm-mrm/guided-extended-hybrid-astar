cmake_minimum_required(VERSION 3.18)
project(freespace_planner)

# we default to Release build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(aduulm_cmake_tools REQUIRED)

aduulm_init_ros2_package()

# Attention these scripts must be made executable manually, if they wish to be executed
install(PROGRAMS
        scripts/simulation.py
        scripts/sim_util.py
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        scripts/sim_config
        scripts/sim_data
        DESTINATION share/${PROJECT_NAME}
        )

# Finish the package creation and generate install() commands
aduulm_create_ros2_package()
setup_git_hooks(py_FORMATTERS "autopep8")


