#include(${CMAKE_CURRENT_LIST_DIR}/CrossCompile.cmake)
include(ExternalProject)

if(NOT YAML_CPP_VERSION)
  set( YAML_CPP_VERSION "0.6.3" )
endif()

if (NOT TARGET yaml-cpp)
  include(ExternalProject)
    ExternalProject_Add(yaml-cpp-ext
      PREFIX ${CMAKE_BINARY_DIR}/yaml_cpp
      URL https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-${YAML_CPP_VERSION}.tar.gz
      CMAKE_ARGS
        ${CROSS_COMPILE}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/yaml-cpp
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DYAML_CPP_BUILD_TESTS=OFF
        )
  add_library(yaml-cpp STATIC IMPORTED GLOBAL)
  add_dependencies(yaml-cpp yaml-cpp-ext)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/yaml-cpp/include)
  set_property(TARGET yaml-cpp PROPERTY INTERFACE_INCLUDE_DIRECTORIES
      ${CMAKE_BINARY_DIR}/yaml-cpp/include
      )

    set_target_properties(yaml-cpp PROPERTIES
      IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/yaml-cpp/lib/${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX})
endif()

set(YAML_CPP_INCLUDE_DIR ${CMAKE_BINARY_DIR}/yaml-cpp/include)
