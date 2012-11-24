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

ELSE()
    SET(PCHSupport_FOUND FALSE)
ENDIF()
if (PCHSupport_FOUND)
    message(STATUS "PCH support found")
else()
    message(STATUS "PCH support not found")
endif()



macro(_pch_get_compile_command out_command _input _output)

    FILE(TO_NATIVE_PATH ${_input} _native_input)
    FILE(TO_NATIVE_PATH ${_output} _native_output)

    separate_arguments(_cxx_flags UNIX_COMMAND "${CMAKE_CXX_FLAGS}")
    string(TOUPPER "CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}" _flags_var_name)
    separate_arguments(_cxx_flags_type UNIX_COMMAND "${${_flags_var_name}}")
    list(APPEND _compile_flags ${_cxx_flags} ${_cxx_flags_type} )
    
    get_filename_component(_absolute_input ${_input} REALPATH)
    get_filename_component(_indir_absolute ${_absolute_input} PATH)
    get_filename_component(_absolute_source_dir "${CMAKE_SOURCE_DIR}" REALPATH)
    file(RELATIVE_PATH _indir_rel "${_absolute_source_dir}" "${_indir_absolute}")
    set(_indir "${CMAKE_SOURCE_DIR}/${_indir_rel}")
    get_property(_indir_includes
        DIRECTORY ${_indir}
        PROPERTY INCLUDE_DIRECTORIES )
    foreach(item ${_indir_includes})
        message("ASDFASDF: ${_indir} ${_input} ${_indir} ${item}")
        list(APPEND _compile_flags "-isystem" ${item} )
    endforeach()

    get_property(_indir_defs
        DIRECTORY ${indir}
        PROPERTY DEFINITIONS)
    separate_arguments(_indir_defs UNIX_COMMAND "${_indir_defs}")
    list(APPEND _compile_flags ${_indir_defs})


    IF(CMAKE_CXX_COMPILER_ARG1)
        # remove leading space in compiler argument
        STRING(REGEX REPLACE "^ +" "" pchsupport_compiler_cxx_arg1 ${CMAKE_CXX_COMPILER_ARG1})

        SET(${out_command}
          ${CMAKE_CXX_COMPILER} ${pchsupport_compiler_cxx_arg1} ${_compile_flags} -x c++-header -o ${_output} ${_input}
          )
    ELSE(CMAKE_CXX_COMPILER_ARG1)
        SET(${out_command}
          ${CMAKE_CXX_COMPILER}  ${_compile_flags} -x c++-header -o ${_output} ${_input}
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
    get_filename_component(_absolute_source_dir "${CMAKE_SOURCE_DIR}" REALPATH)
    file(RELATIVE_PATH _source_relative_header_file "${_absolute_source_dir}" "${_absolute_header_file}")
    get_filename_component(_pch_file "${CMAKE_BINARY_DIR}/${_source_relative_header_file}" REALPATH)
    
    string(REPLACE "/" "_" _pch_target ${_source_relative_header_file})
    string(REPLACE "." "_" _pch_target ${_pch_target})
    set(_pch_target "${_pch_target}_pch")

    _pch_get_target_compile_flags(_target_cflags ${_pch_file} ${_dowarn})
    set_property(
        TARGET ${_targetName}
        APPEND_STRING PROPERTY COMPILE_FLAGS ${_target_cflags}
    )
    
    # Does add_dependencies guarantee that the objects are rebuilt? Probably not, 
    # add dependencies directly to the source files just in case

    add_dependencies(${_targetName} ${_pch_target} )
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
    get_filename_component(_absolute_source_dir "${CMAKE_SOURCE_DIR}" REALPATH)
    file(RELATIVE_PATH _source_relative_header_file "${_absolute_source_dir}" "${_absolute_header_file}")
    message("ASDFASDF: ${_absolute_header_file}")
    get_filename_component(_pch_file "${CMAKE_BINARY_DIR}/${_source_relative_header_file}" REALPATH)
    
    set(_output "${_pch_file}.gch")
    get_filename_component(_outdir ${_output} PATH)
    
    string(REPLACE "/" "_" _pch_target ${_source_relative_header_file})
    string(REPLACE "." "_" _pch_target ${_pch_target})
    set(_pch_target "${_pch_target}_pch")

    if (NOT TARGET ${_pch_target})

        set(_dephelp "${_pch_target}_dephelp")

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
