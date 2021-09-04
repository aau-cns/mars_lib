#include(${CMAKE_CURRENT_LIST_DIR}/CrossCompile.cmake)
include(ExternalProject)

if(NOT Sophus_VERSION)
  set( Sophus_VERSION "1.0.0" )
endif()

#if (NOT TARGET Kindr)
include(ExternalProject)
externalproject_add(Sophus-ext
    PREFIX ${CMAKE_BINARY_DIR}/Sophus
    GIT_REPOSITORY "https://github.com/strasdat/Sophus.git"
    GIT_TAG "13fb3288311485dc94e3226b69c9b59cd06ff94e"
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS
    ${CROSS_COMPILE}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/Sophus
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    )


add_library(Sophus INTERFACE IMPORTED GLOBAL)
add_dependencies(Sophus Sophus-ext)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/Sophus/include)
set_property(TARGET Sophus PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    ${CMAKE_BINARY_DIR}/Sophus/include
    )
#endif()

set(SOPHUS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/Sophus/include)

