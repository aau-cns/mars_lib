
# 
# Configuration for all sub-projects
# 

# Generate version-header
configure_file(version.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/${META_PROJECT_NAME}/${META_PROJECT_NAME}-version.h)

#
# Configuration of basic libraries
#
find_package(Eigen REQUIRED)
find_package(yaml-cpp REQUIRED)
#find_package(kindr REQUIRED)
#find_package(Sophus REQUIRED)

message(" **************************************")
message(" --- Main Dependencies:")
message(" * Eigen: ${EIGEN3_INCLUDE_DIR}, ${Eigen_VERSION} ")
message(" * YAML_CPP: ${YAML_CPP_INCLUDE_DIR}, ${YAML_CPP_VERSION} ")
#message(" * kindr: ${KINDR_INCLUDE_DIR}, ${kindr_VERSION} ")
#message(" * Sophus: ${SOPHUS_INCLUDE_DIR}, ${Sophus_VERSION} ")
message(" **************************************")
# 
# Sub-projects
# 

# Libraries
set(IDE_FOLDER "")
add_subdirectory(mars)

# Examples
set(IDE_FOLDER "Examples")
add_subdirectory(examples)

# Tests
if(OPTION_BUILD_TESTS)
    set(IDE_FOLDER "Tests")
    add_subdirectory(tests)
endif()


# 
# Deployment
# 

# Deploy generated headers
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/${META_PROJECT_NAME} DESTINATION include COMPONENT dev)
