
#
# External dependencies
#

find_package(${META_PROJECT_NAME} REQUIRED HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../")

#
# Executable name and options
#

# Target name
set(target mars-e2e-test)
set(target_lib mars)
message(STATUS "Test ${target}")


#
# Sources
#

set(sources
    main.cpp
    mars_e2e_imu_prop.cpp
    mars_e2e_imu_pose_update.cpp
    mars_e2e_imu_pose_ooo_rework.cpp
    mars_e2e_imu_pose_outlier.cpp
    mars_e2e_imu_pose_update_perf.cpp
    mars_e2e_imu_prop_empty_updates.cpp
)


#
# Create executable
#

# Build executable
add_executable(${target}
    ${sources}
)

# Create namespaced alias
add_executable(${META_PROJECT_NAME}::${target} ALIAS ${target})


#
# Project options
#

set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
    FOLDER "${IDE_FOLDER}"
)


#
# Include directories
#

target_include_directories(${target}
    PRIVATE
    ${DEFAULT_INCLUDE_DIRECTORIES}
)


#
# Libraries
#

target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LIBRARIES}
    ${META_PROJECT_NAME}::${target_lib}
    gmock-dev
)


#
# Compile definitions
#

target_compile_definitions(${target}
    PRIVATE
    ${DEFAULT_COMPILE_DEFINITIONS}
)


#
# Compile options
#

target_compile_options(${target}
    PRIVATE
    ${DEFAULT_COMPILE_OPTIONS}
)


#
# Linker options
#

target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LINKER_OPTIONS}
)

