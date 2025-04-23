
# Check if examples are enabled
if(NOT OPTION_BUILD_EXAMPLES)
    return()
endif()

# Example applications
add_subdirectory(mars_cmd)
add_subdirectory(mars_thl)
add_subdirectory(mars_insane_dataset)
