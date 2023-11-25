project(freespace_planner)

set(INSTALL_PATH ${SANDBOX_PATH}/install/${PROJECT_NAME})
set(PYTHON_INST_PATH ${INSTALL_PATH}/lib/python3.10/site-packages)

# Install ini file to keep layout
if(EXISTS scripts/.simulation.imviz.ini)
    install(FILES
            scripts/.simulation.imviz.ini
            DESTINATION ${INSTALL_PATH}/lib/${PROJECT_NAME}
    )
else()
endif()

# install only symlinks
set(CMAKE_INSTALL_MODE ABS_SYMLINK)

# TODO Do symlink install for convenience

# Attention these scripts must be made executable manually, if they wish to be executed
install(PROGRAMS
        scripts/simulation.py
        scripts/sim_util.py
        DESTINATION ${INSTALL_PATH}/lib/${PROJECT_NAME}
)

# Install simulation data folder
# Install config folder
install(DIRECTORY
        scripts/sim_config
        scripts/sim_data
        DESTINATION ${INSTALL_PATH}/share/${PROJECT_NAME}
)