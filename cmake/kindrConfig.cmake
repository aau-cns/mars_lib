#include(${CMAKE_CURRENT_LIST_DIR}/CrossCompile.cmake)
include(ExternalProject)

if(NOT kindr_VERSION)
  set( kindr_VERSION "1.0" )
endif()

#if (NOT TARGET Kindr)
include(ExternalProject)
externalproject_add(kindr-ext
    PREFIX ${CMAKE_BINARY_DIR}/kindr
    GIT_REPOSITORY "https://github.com/ethz-asl/kindr.git"
    GIT_TAG "1639d2c85cfb088f96b1c9be2e628fafcaa87607"
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
    ${CROSS_COMPILE}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/kindr
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    )


add_library(kindr INTERFACE IMPORTED GLOBAL)
add_dependencies(kindr kindr-ext)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/kindr/include)
set_property(TARGET kindr PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    ${CMAKE_BINARY_DIR}/kindr/include
    )
#endif()

set(KINDR_INCLUDE_DIR ${CMAKE_BINARY_DIR}/kindr/include)

