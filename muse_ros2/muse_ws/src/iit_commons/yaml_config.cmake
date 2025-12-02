# Disable by default the building of YAML facilities.
#  The user has to explicitly choose it, if he wants (eg. use 'ccmake ..')
set(BUILD_LIBYAML off CACHE BOOL "Build the lib with yaml-based configuration facilities")

if(BUILD_LIBYAML)
    set(YAML_SRCS_DIR src/yaml_config)
    set(YAML_LIB_NAME ${LIB_NAME}-yamlcfg)
    set(SOURCES ${YAML_SRCS_DIR}/yaml_config.cpp)

    # Add library
    add_library(${YAML_LIB_NAME} SHARED ${SOURCES})
    install(TARGETS ${YAML_LIB_NAME} LIBRARY DESTINATION ${LIB_INSTALL_ROOT})

    add_executable(yaml-example ${YAML_SRCS_DIR}/example.cpp)
    target_link_libraries(yaml-example ${YAML_LIB_NAME} yaml-cpp ${LIB_NAME})

endif(BUILD_LIBYAML)
