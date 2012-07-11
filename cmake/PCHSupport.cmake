# taken from http://public.kitware.com/Bug/view.php?id=1260 and slightly adjusted
# ...by opencv, and then modified again by cauv

# - Try to find precompiled headers support for GCC 3.4 and 4.x
# Once done this will define:
#
# Variable:
#   PCHSupport_FOUND
#
# Macro:
#   ADD_PRECOMPILED_HEADER  _targetName _input

IF(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    EXEC_PROGRAM(
        ${CMAKE_CXX_COMPILER}
        ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
        OUTPUT_VARIABLE gcc_compiler_version)
    IF(gcc_compiler_version MATCHES "4\\.[0,2-9](\\.[0-9])?")
        SET(PCHSupport_FOUND TRUE)
    ENDIF()

    SET(_PCH_include_prefix "-I")
    SET(_PCH_isystem_prefix "-isystem")

ELSEIF(CMAKE_GENERATOR MATCHES "^Visual.*$")
    SET(PCHSupport_FOUND TRUE)
    SET(_PCH_include_prefix "/I")
    SET(_PCH_isystem_prefix "/I")
ELSE()
    SET(PCHSupport_FOUND FALSE)
ENDIF()
if (PCHSupport_FOUND)
    message(STATUS "PCH support found")
else()
    message(STATUS "PCH support not found")
endif()

MACRO(_PCH_GET_COMPILE_FLAGS _out_compile_flags)

    STRING(TOUPPER "CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}" _flags_var_name)
    SET(${_out_compile_flags} ${${_flags_var_name}} )

    IF(CMAKE_COMPILER_IS_GNUCXX)

        GET_TARGET_PROPERTY(_targetType ${_PCH_current_target} TYPE)
        IF(${_targetType} STREQUAL SHARED_LIBRARY AND NOT WIN32)
            LIST(APPEND ${_out_compile_flags} "-fPIC")
        ENDIF()

    ELSE()
        ## TODO ... ? or does it work out of the box
    ENDIF()

    GET_DIRECTORY_PROPERTY(DIRINC INCLUDE_DIRECTORIES )
    FOREACH(item ${DIRINC})
        LIST(APPEND ${_out_compile_flags} "${_PCH_isystem_prefix}" "${item}")
    ENDFOREACH()

    GET_DIRECTORY_PROPERTY(_directory_flags DEFINITIONS)
    #MESSAGE("_directory_flags ${_directory_flags} ${_global_definitions}" )
    LIST(APPEND ${_out_compile_flags} ${_directory_flags})
    LIST(APPEND ${_out_compile_flags} ${CMAKE_CXX_FLAGS})

    SEPARATE_ARGUMENTS(${_out_compile_flags})

ENDMACRO(_PCH_GET_COMPILE_FLAGS)


macro(_pch_write_pchdep_cxx _dephelp_cpp _dephelp _include_file)

endmacro()


macro(_pch_get_compile_command out_command _input _output)

    FILE(TO_NATIVE_PATH ${_input} _native_input)
    FILE(TO_NATIVE_PATH ${_output} _native_output)

    IF(CMAKE_CXX_COMPILER_ARG1)
        # remove leading space in compiler argument
        STRING(REGEX REPLACE "^ +" "" pchsupport_compiler_cxx_arg1 ${CMAKE_CXX_COMPILER_ARG1})

        SET(${out_command}
          ${CMAKE_CXX_COMPILER} ${pchsupport_compiler_cxx_arg1} ${_compile_FLAGS} -x c++-header -o ${_output} ${_input}
          )
    ELSE(CMAKE_CXX_COMPILER_ARG1)
        SET(${out_command}
          ${CMAKE_CXX_COMPILER}  ${_compile_FLAGS} -x c++-header -o ${_output} ${_input}
          )
    ENDIF(CMAKE_CXX_COMPILER_ARG1)

ENDMACRO()


macro(_pch_get_target_compile_flags _cflags _pch_file _dowarn )

    # for use with distcc and gcc >4.0.1 if preprocessed files are accessible
    # on all remote machines set
    # PCH_ADDITIONAL_COMPILER_FLAGS to -fpch-preprocess
    # if you want warnings for invalid header files (which is very inconvenient
    # if you have different versions of the headers for different build types
    # you may set _pch_dowarn
    if (_dowarn)
        set(${_cflags} "${PCH_ADDITIONAL_COMPILER_FLAGS} -include \"${_pch_file}\" -Winvalid-pch " )
    else ()
        set(${_cflags} "${PCH_ADDITIONAL_COMPILER_FLAGS} -include \"${_pch_file}\" " )
    endif ()

endmacro()


macro(use_precompiled_header _targetName _header_file )

    if (PCHSupport_FOUND AND CAUV_USE_PRECOMPILED_HEADERS)

    IF(ARGN STREQUAL "0")
        SET(_dowarn 0)
    ELSE()
        SET(_dowarn 1)
    ENDIF()
    
    get_filename_component(_absolute_header_file "${_header_file}" REALPATH)
    file(RELATIVE_PATH _source_relative_header_file "${CMAKE_SOURCE_DIR}" "${_absolute_header_file}")
    get_filename_component(_pch_file "${CMAKE_BINARY_DIR}/${_source_relative_header_file}" REALPATH)

    _pch_get_target_compile_flags(_target_cflags ${_pch_file} ${_dowarn})
    set_property(
        TARGET ${_targetName}
        APPEND_STRING PROPERTY COMPILE_FLAGS ${_target_cflags}
    )
    
    # Does add_dependencies guarantee that the objects are rebuilt? Probably not, 
    # add dependencies directly to the source files just in case

    #add_custom_target(${_targetName}_pch
    #  DEPENDS ${_pch_output_to_use}
    #  )
    #add_dependencies(${_targetName} ${_targetName}_pch )
    get_property( _targetSources
        TARGET ${_targetName}
        PROPERTY SOURCES )
    foreach(_source ${_targetSources})
        set_property(
            SOURCE "${_source}"
            APPEND PROPERTY OBJECT_DEPENDS ${_pch_file}.gch
            )
    endforeach()

    endif()

endmacro()


macro(add_precompiled_header _target _header_file)

    if (PCHSupport_FOUND AND CAUV_USE_PRECOMPILED_HEADERS)

    if(ARGN STREQUAL "0")
        set(_dowarn 0)
    else()
        set(_dowarn 1)
    endif()

    get_filename_component(_name ${_header_file} NAME)
    get_filename_component(_path ${_header_file} PATH)

    get_filename_component(_absolute_header_file "${_header_file}" REALPATH)
    file(RELATIVE_PATH _source_relative_header_file "${CMAKE_SOURCE_DIR}" "${_absolute_header_file}")
    get_filename_component(_pch_file "${CMAKE_BINARY_DIR}/${_source_relative_header_file}" REALPATH)
    
    set(_output "${_pch_file}.gch")
    get_filename_component(_outdir ${_output} PATH)
    
    string(REPLACE "/" "_" _pch_target ${_source_relative_header_file})
    string(REPLACE "." "_" _pch_target ${_pch_target})
    
    set(_pch_target "${_pch_target}_pch")

    if (NOT TARGET ${_pch_target})

        set(_dephelp "${_pch_target}_dephelp")


        _pch_get_compile_flags(_compile_FLAGS)
        _pch_get_compile_command(_command  ${_header_file} ${_output} )
        
        file(COPY "${_header_file}" DESTINATION "${_outdir}")
     
        set(_dephelp_cpp ${_output}.dephelp.cpp)
        FILE(WRITE
          ${_dephelp_cpp}.in
          "#include \"${_pch_file}\"
          ")
        configure_file(${_dephelp_cpp}.in ${_dephelp_cpp} COPYONLY)
        add_library(${_dephelp} STATIC "${_dephelp_cpp}" )

        add_custom_command(
          OUTPUT "${_output}"
          COMMAND ${_command}
          DEPENDS ${_dephelp}
          VERBATIM)

        add_custom_target(
            ${_pch_target}
            DEPENDS "${_output}"
        )

    endif()

    use_precompiled_header ("${_target}" "${_header_file}")

    endif()

endmacro() 
