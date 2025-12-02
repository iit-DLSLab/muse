# The building of Euler-angles facilities is disabled by default
#  The user has to explicitly choose it, if needed (eg. use 'ccmake ..')

set(BUILD_LIBEULER off CACHE BOOL "Build the lib with Euler-angles facilities")

if(BUILD_LIBEULER)
    set(EULER_SRCS_DIR src/euler_angles)
    set(EULER_LIB_NAME ${LIB_NAME}-euler)
    set(SOURCES ${EULER_SRCS_DIR}/transforms_internal.cpp)

    # Add library
    add_library(${EULER_LIB_NAME} SHARED ${SOURCES})
    install(TARGETS ${EULER_LIB_NAME} LIBRARY DESTINATION ${LIB_INSTALL_ROOT})

    # Example
    add_executable(euler-example ${EULER_SRCS_DIR}/example.cpp)
    target_link_libraries(euler-example ${EULER_LIB_NAME})
endif(BUILD_LIBEULER)
