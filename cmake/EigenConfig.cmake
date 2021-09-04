#include(${CMAKE_CURRENT_LIST_DIR}/CrossCompile.cmake)
include(ExternalProject)

if(NOT Eigen_VERSION)
  set( Eigen_VERSION "3.3.7" )
endif()

if (NOT TARGET Eigen)
  include(ExternalProject)
  externalproject_add(Eigen-ext
      PREFIX ${CMAKE_BINARY_DIR}/Eigen
      URL https://gitlab.com/libeigen/eigen/-/archive/${Eigen_VERSION}/eigen-${Eigen_VERSION}.tar
      CMAKE_ARGS
      ${CROSS_COMPILE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/Eigen
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      )
  add_library(Eigen INTERFACE IMPORTED GLOBAL)
  add_dependencies(Eigen Eigen-ext)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/Eigen/include/eigen3)
  set_property(TARGET Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES
      ${CMAKE_BINARY_DIR}/Eigen/include/eigen3
      )
endif()

set(EIGEN3_INCLUDE_DIR ${CMAKE_BINARY_DIR}/Eigen/include/eigen3)
# for g2on
set(EIGEN_INCLUDE_DIR ${CMAKE_BINARY_DIR}/Eigen/include/eigen3)
