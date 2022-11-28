
#
# External dependencies
#

# find_package(THIRDPARTY REQUIRED)


#
# Library name and options
#

# Target name
set(target mars)

# Exit here if required dependencies are not met
message(STATUS "Lib ${target}")

# Set API export file and macro
string(MAKE_C_IDENTIFIER ${target} target_id)
string(TOUPPER ${target_id} target_id)
set(feature_file         "include/${target}/${target}_features.h")
set(export_file          "include/${target}/${target}_export.h")
set(template_export_file "include/${target}/${target}_api.h")
set(export_macro         "${target_id}_API")


#
# Sources
#

set(include_path "${CMAKE_CURRENT_SOURCE_DIR}/include/${target}")
set(include_local_path "${CMAKE_CURRENT_SOURCE_DIR}/include_local/${target}")
set(source_path  "${CMAKE_CURRENT_SOURCE_DIR}/source")

set(headers
    ${include_path}/time.h
    ${include_path}/buffer.h
    ${include_path}/core_state.h
    ${include_path}/core_logic.h
    ${include_path}/sensor_manager.h
    ${include_path}/nearest_cov.h
    ${include_path}/ekf.h
    ${include_path}/m_perf.h
    ${include_path}/general_functions/utils.h
    ${include_path}/general_functions/progress_indicator.h
    ${include_path}/type_definitions/base_states.h
    ${include_path}/type_definitions/buffer_entry_type.h
    ${include_path}/type_definitions/buffer_data_type.h
    ${include_path}/type_definitions/core_state_type.h
    ${include_path}/type_definitions/core_type.h
    ${include_path}/type_definitions/mars_types.h
    ${include_path}/sensors/sensor_abs_class.h
    ${include_path}/sensors/update_sensor_abs_class.h
    ${include_path}/sensors/sensor_interface.h
    ${include_path}/sensors/bind_sensor_data.h
    ${include_path}/sensors/measurement_base_class.h
    ${include_path}/sensors/measurement_interface.h
    ${include_path}/sensors/imu/imu_sensor_class.h
    ${include_path}/sensors/imu/imu_measurement_type.h
    ${include_path}/sensors/position/position_measurement_type.h
    ${include_path}/sensors/position/position_sensor_class.h
    ${include_path}/sensors/position/position_sensor_state_type.h
    ${include_path}/sensors/pose/pose_measurement_type.h
    ${include_path}/sensors/pose/pose_sensor_class.h
    ${include_path}/sensors/pose/pose_sensor_state_type.h
    ${include_path}/sensors/gps/gps_conversion.h
    ${include_path}/sensors/gps/gps_measurement_type.h
    ${include_path}/sensors/gps/gps_sensor_class.h
    ${include_path}/sensors/gps/gps_sensor_state_type.h
    ${include_path}/sensors/gps_w_vel/gps_w_vel_measurement_type.h
    ${include_path}/sensors/gps_w_vel/gps_w_vel_sensor_class.h
    ${include_path}/sensors/gps_w_vel/gps_w_vel_sensor_state_type.h
    ${include_path}/sensors/mag/mag_measurement_type.h
    ${include_path}/sensors/mag/mag_sensor_class.h
    ${include_path}/sensors/mag/mag_sensor_state_type.h
    ${include_path}/sensors/mag/mag_utils.h
    ${include_path}/sensors/pressure/pressure_conversion.h
    ${include_path}/sensors/pressure/pressure_measurement_type.h
    ${include_path}/sensors/pressure/pressure_sensor_class.h
    ${include_path}/sensors/pressure/pressure_sensor_state_type.h
    ${include_path}/sensors/pressure/pressure_utils.h
    ${include_path}/sensors/vision/vision_measurement_type.h
    ${include_path}/sensors/vision/vision_sensor_class.h
    ${include_path}/sensors/vision/vision_sensor_state_type.h
    ${include_path}/sensors/bodyvel/bodyvel_measurement_type.h
    ${include_path}/sensors/bodyvel/bodyvel_sensor_class.h
    ${include_path}/sensors/bodyvel/bodyvel_sensor_state_type.h
    ${include_path}/sensors/attitude/attitude_conversion.h
    ${include_path}/sensors/attitude/attitude_measurement_type.h
    ${include_path}/sensors/attitude/attitude_sensor_class.h
    ${include_path}/sensors/attitude/attitude_sensor_state_type.h
    ${include_path}/sensors/empty/empty_measurement_type.h
    ${include_path}/sensors/empty/empty_sensor_class.h
    ${include_path}/sensors/empty/empty_sensor_state_type.h
    ${include_path}/data_utils/read_csv.h
    ${include_path}/data_utils/write_csv.h
    ${include_path}/data_utils/read_sim_data.h
    ${include_path}/data_utils/read_imu_data.h
    ${include_path}/data_utils/read_pose_data.h
    ${include_path}/data_utils/read_vision_data.h
    ${include_path}/data_utils/read_position_data.h
    ${include_path}/data_utils/read_gps_w_vel_data.h
    ${include_path}/data_utils/read_gps_data.h
    ${include_path}/data_utils/read_mag_data.h
    ${include_path}/data_utils/read_baro_data.h
    ${include_path}/data_utils/filesystem.h
)

set(sources
    ${source_path}/time.cpp
    ${source_path}/buffer_entry_type.cpp
    ${source_path}/buffer.cpp
    ${source_path}/core_logic.cpp
    ${source_path}/core_state.cpp
    ${source_path}/core_state_calc_q.cpp
    ${source_path}/nearest_cov.cpp
    ${source_path}/ekf.cpp
    ${source_path}/m_perf.cpp
    ${source_path}/utils.cpp
    ${include_path}/sensors/gps/gps_conversion.cpp
    ${include_path}/sensors/pressure/pressure_conversion.cpp
    ${include_path}/sensors/pressure/pressure_utils.cpp
    ${include_path}/sensors/mag/mag_utils.cpp
    ${include_path}/general_functions/progress_indicator.cpp
    ${include_path}/data_utils/filesystem.cpp
    #${source_path}/sensor_manager.cpp
)

# Group source files
set(header_group "Header Files (API)")
set(source_group "Source Files")
source_group_by_path(${include_path} "\\\\.h$|\\\\.hpp$"
    ${header_group} ${headers})
source_group_by_path(${source_path}  "\\\\.cpp$|\\\\.c$|\\\\.h$|\\\\.hpp$"
    ${source_group} ${sources})


#
# Create library
#

# Build library
add_library(${target}
    ${sources}
    ${headers}
)

# Create namespaced alias
add_library(${META_PROJECT_NAME}::${target} ALIAS ${target})

# Export library for downstream projects
export(TARGETS ${target} NAMESPACE ${META_PROJECT_NAME}:: FILE ${PROJECT_BINARY_DIR}/cmake/${target}/${target}-export.cmake)

# Create feature detection header
# Compilers: https://cmake.org/cmake/help/v3.1/variable/CMAKE_LANG_COMPILER_ID.html#variable:CMAKE_%3CLANG%3E_COMPILER_ID
# Feature: https://cmake.org/cmake/help/v3.1/prop_gbl/CMAKE_CXX_KNOWN_FEATURES.html

# Check for availability of module; use pre-generated version if not found
if (WriterCompilerDetectionHeaderFound)
    write_compiler_detection_header(
        FILE ${feature_file}
        PREFIX ${target_id}
        COMPILERS AppleClang Clang GNU MSVC
        FEATURES cxx_alignas cxx_alignof cxx_constexpr cxx_final cxx_noexcept cxx_nullptr cxx_sizeof_member cxx_thread_local
        VERSION 3.2
    )
else()
    file(
        COPY ${PROJECT_SOURCE_DIR}/source/codegeneration/${target}_features.h
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/${target}
        USE_SOURCE_PERMISSIONS
    )
endif()

# Create API export header
generate_export_header(${target}
    EXPORT_FILE_NAME  ${export_file}
    EXPORT_MACRO_NAME ${export_macro}
)
generate_template_export_header(${target}
    ${target_id}
    ${template_export_file}
)


#
# Project options
#

set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
    FOLDER "${IDE_FOLDER}"
    VERSION ${META_VERSION}
    SOVERSION ${META_VERSION_MAJOR}
)


#
# Include directories
#

target_include_directories(${target}
    PRIVATE
    ${PROJECT_BINARY_DIR}/source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include_local
    ${CMAKE_CURRENT_BINARY_DIR}/include

    PUBLIC
    ${DEFAULT_INCLUDE_DIRECTORIES}

    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


#
# Libraries
#

target_link_libraries(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_LIBRARIES}
    Eigen
    yaml-cpp
    Boost
    #kindr
    #Sophus
    INTERFACE
)


#
# Compile definitions
#

target_compile_definitions(${target}
    PRIVATE

    PUBLIC
    $<$<NOT:$<BOOL:${BUILD_SHARED_LIBS}>>:${target_id}_STATIC_DEFINE>
    ${DEFAULT_COMPILE_DEFINITIONS}

    INTERFACE
)


#
# Compile options
#

target_compile_options(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_COMPILE_OPTIONS}

    INTERFACE
)


#
# Linker options
#

target_link_libraries(${target}
    PRIVATE

    PUBLIC
    ${DEFAULT_LINKER_OPTIONS}

    INTERFACE
)


#
# Target Health
#

perform_health_checks(
    ${target}
    ${sources}
    ${headers}
)


#
# Deployment
#

# Library
install(TARGETS ${target}
    EXPORT  "${target}-export"            COMPONENT dev
    RUNTIME DESTINATION ${INSTALL_BIN}    COMPONENT runtime
    LIBRARY DESTINATION ${INSTALL_SHARED} COMPONENT runtime
    ARCHIVE DESTINATION ${INSTALL_LIB}    COMPONENT dev
)

# Header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${target} DESTINATION ${INSTALL_INCLUDE}
    COMPONENT dev
)

# Generated header files
install(DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/include/${target} DESTINATION ${INSTALL_INCLUDE}
    COMPONENT dev
)

# CMake config
install(EXPORT ${target}-export
    NAMESPACE   ${META_PROJECT_NAME}::
    DESTINATION ${INSTALL_CMAKE}/${target}
    COMPONENT   dev
)
