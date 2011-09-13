#set (CMAKE_VERBOSE_MAKEFILE on)

cmake_minimum_required (VERSION 2.8)
project (GeminiSDK)

set (${PROJECT_NAME}_MAJOR_VERSION 0)
set (${PROJECT_NAME}_MINOR_VERSION 0)
set (${PROJECT_NAME}_PATCH_VERSION 1)
set (${PROJECT_NAME}_VERSION "${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION}")

# set the directory that has tools for finding packages etc...
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/Tools/cmake/Modules/")

# build static library
set (BUILD_SHARED_LIBS ON)

# include source directory
#include_directories (${CMAKE_SOURCE_DIR}/Src/SDK)

# output directories
set (LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Single directory for all libraries")
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Single directory for all libraries")
mark_as_advanced (LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH)

############################### FOLDERS TO BUILD ###############################

# SDK
include (${CMAKE_SOURCE_DIR}/Src/SDK/CMakeLists.txt)

# Source files to build.
foreach(SRC_FILE ${FILES})
	list(APPEND GEMINI_SDK_SRC ${CMAKE_SOURCE_DIR}/Src/SDK/${SRC_FILE}.cpp)
endforeach(SRC_FILE)

# Headers to include when installing.
list(APPEND GEMINI_SDK_INSTALL_HDRS ${CMAKE_SOURCE_DIR}/Src/SDK/GeminiStructuresPublic.h)
list(APPEND GEMINI_SDK_INSTALL_HDRS ${CMAKE_SOURCE_DIR}/Src/SDK/GeminiStructures.h)
list(APPEND GEMINI_SDK_INSTALL_HDRS ${CMAKE_SOURCE_DIR}/Src/SDK/GeminiCommsPublic.h)
list(APPEND GEMINI_SDK_INSTALL_HDRS ${CMAKE_SOURCE_DIR}/Src/SDK/DataTypes.h)
install(FILES ${GEMINI_SDK_INSTALL_HDRS} DESTINATION include/${PROJECT_NAME})
		  

# DRIVER
# nothing yet.....

########################### CREATE & INSTALL LIBRARY ###########################
add_library (${PROJECT_NAME} ${GEMINI_SDK_SRC})
# this variale should be changeable depending on the linux sytem - there's an
# explanation about it on google. I'll need to look into it later.
# set (CMAKE_INSTALL_SO_NO_EXE "1")
if (WIN32)
	target_link_libraries(${PROJECT_NAME} setupapi ws2_32 Iphlpapi)
endif (WIN32)
add_definitions(-DUNICODE -D_UNICODE)
set_target_properties (${PROJECT_NAME} PROPERTIES
							  VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION} 
							  SOVERSION ${${PROJECT_NAME}_MAJOR_VERSION})

install (TARGETS ${PROJECT_NAME} DESTINATION lib)
######################## BUILD & INSTALL DOCUMENTATION #########################
#find_package(Doxygen)
#
#if(DOXYGEN_FOUND)
#	find_file(DOXYFILE_IN "Doxyfile.in"
#			PATHS "${CMAKE_CURRENT_SOURCE_DIR}" "${SOURCE_DIR}" "${CMAKE_ROOT}/Modules/")
#
#	include(FindPackageHandleStandardArgs)
#	find_package_handle_standard_args(Doxyfile.in DEFAULT_MSG DOXYFILE_IN)
#endif()
#
#if(DOXYGEN_FOUND AND DOXYFILE_IN)
#	add_custom_target(doxygen ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
#
#	set(DOXYFILE_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/doc")
#	set(DOXYFILE_HTML_DIR "html")
#	set(DOXYFILE_LATEX "NO")
#	set(DOXYFILE_PDFLATEX "NO")
#	set(DOXYFILE_DOT "NO")	
#
#	set_property(DIRECTORY APPEND PROPERTY
#					 ADDITIONAL_MAKE_CLEAN_FILES "${DOXYFILE_OUTPUT_DIR}/${DOXYFILE_HTML_DIR}")	
#	
#	configure_file(${DOXYFILE_IN} Doxyfile ESCAPE_QUOTES IMMEDIATE @ONLY)
#
#	get_target_property(DOC_TARGET doc TYPE)
#	if(NOT DOC_TARGET)
#		add_custom_target(doc)
#	endif()
#			
#	add_dependencies(doc doxygen)
#endif()
#
#install(DIRECTORY ${DOXYFILE_OUTPUT_DIR}/
#		  DESTINATION doc/${PROJECT_NAME}/v${${PROJECT_NAME}_MAJOR_VERSION}_${${PROJECT_NAME}_MINOR_VERSION}_${${PROJECT_NAME}_PATCH_VERSION})

