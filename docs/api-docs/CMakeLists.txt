
# 
# Find doxygen
# 

find_package(Doxygen)
if(NOT DOXYGEN_FOUND)
    message(STATUS "Disabled generation of doxygen documentation (missing doxygen).")
    return()
endif()


# 
# Target name
# 

set(target api-docs)
message(STATUS "Doc ${target}")

set(DEPENDENCIES "${META_PROJECT_NAME}::mars")
# 
# Input file
# 

set(doxyfile_in doxyfile.in)


# 
# Create documentation
# 

# Set project variables
set(doxyfile            "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
set(doxyfile_directory  "${CMAKE_CURRENT_BINARY_DIR}/html")
set(doxyfile_html       "${doxyfile_directory}/index.html")

# Get filename and path of doxyfile
get_filename_component(name ${doxyfile_in} NAME)
get_filename_component(path ${doxyfile_in} PATH)
if(NOT path)
    set(path ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# Configure doxyfile (if it is a real doxyfile already, it should simply copy the file)
set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
configure_file(${doxyfile_in} ${doxyfile})

# Invoke doxygen
add_custom_command(
    OUTPUT              ${doxyfile_html}
    DEPENDS             ${doxyfile} ${DEPENDENCIES}
    WORKING_DIRECTORY   ${path}
    COMMAND             ${CMAKE_COMMAND} -E copy_directory ${path} ${doxyfile_directory} # ToDO, configure doxygen to use source as is
    COMMAND             ${DOXYGEN} \"${doxyfile}\"
    COMMENT             "Creating doxygen documentation."
)

# Declare target
add_custom_target(${target} ALL DEPENDS ${doxyfile_html})
add_dependencies(docs ${target})


# 
# Deployment
# 

install(
    DIRECTORY ${doxyfile_directory}
    DESTINATION ${INSTALL_DOC}
    COMPONENT docs
)
