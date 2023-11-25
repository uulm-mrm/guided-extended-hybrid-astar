set(MODULE_NAME "path_planner_lib")

# TODO adapt to public_repo install
set(CONFIG_SRC ${PROJECT_SOURCE_DIR}/config)
set(DATA_SRC ${PROJECT_SOURCE_DIR}/data)
set(PYTHON_SRC ${PROJECT_SOURCE_DIR}/python/)

set(CONFIG_INST ${INSTALL_PATH}/share/${PROJECT_NAME}/config)
set(DATA_INST ${INSTALL_PATH}/share/${PROJECT_NAME}/)
#set(PYTHON_MODULE_INST_DIR ${PYTHON_INST_PATH}/${MODULE_NAME})
set(PYTHON_MODULE_INST_DIR ${PYTHON_INST_PATH}/)

# Install data folder
install(DIRECTORY
        ${DATA_SRC}
        DESTINATION ${DATA_INST}/
        )

# Install config file as symlink
#INSTALL(CODE "execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CONFIG_SRC} ${CONFIG_INST} )")

# Copy to install path
install(DIRECTORY
        ${PYTHON_SRC}
        DESTINATION ${PYTHON_MODULE_INST_DIR}/
)

find_package(Python COMPONENTS Interpreter REQUIRED)
set(_pip_args "--user")
message("PYTHON_MODULE_INST_DIR ${PYTHON_MODULE_INST_DIR}")
INSTALL(CODE "execute_process( COMMAND ${Python_EXECUTABLE} -m pip install ${_pip_args} -e ${PYTHON_MODULE_INST_DIR})")

# Install python part of lib as symlink
#message("PYTHON_SRC ${PYTHON_SRC}")
#message("PYTHON_MODULE_INST_DIR ${PYTHON_MODULE_INST_DIR}")
#INSTALL(CODE "execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${PYTHON_SRC} ${PYTHON_MODULE_INST_DIR} )")
