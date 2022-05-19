if(NOT BOOST_VERSION)
  set( BOOST_VERSION "1.74.0" )
endif()

string(REPLACE "." "_" BOOST_VERSION_UNDERSCORE ${BOOST_VERSION})

set( boost_URL "https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_UNDERSCORE}.tar.gz" )
set( boost_install_DIR ${CMAKE_BINARY_DIR}/boost )


if (NOT TARGET Boost)
  include(ExternalProject)
    ExternalProject_Add(boost-ext
      PREFIX ${CMAKE_BINARY_DIR}/boost
      URL ${boost_URL}
      
      CONFIGURE_COMMAND ./bootstrap.sh
            --with-libraries=math
            #--with-libraries=filesystem
            #--with-libraries=system
            #--with-libraries=date_time
            --prefix=${boost_install_DIR}
      BUILD_COMMAND ./b2 link=static variant=release threading=multi runtime-link=static install
      BUILD_IN_SOURCE true
      INSTALL_COMMAND ""
      INSTALL_DIR ${boost_install_DIR}
  )

add_library(Boost STATIC IMPORTED GLOBAL)
add_dependencies(Boost boost-ext)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/boost/include)
set_property(TARGET Boost PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    ${CMAKE_BINARY_DIR}/boost/include
)


set_target_properties(Boost PROPERTIES IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/boost/lib/libboost_math_c99.a)
#${CMAKE_BINARY_DIR}/boost/lib/${CMAKE_STATIC_LIBRARY_PREFIX}boost${CMAKE_STATIC_LIBRARY_SUFFIX})
endif()

set(BOOST_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/boost/include)
