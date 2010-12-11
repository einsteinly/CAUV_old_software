project (utility)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${AUV_BINARY_DIR}/bin")

# everything needs boost: 
find_package (Boost 1.43 REQUIRED)
set (LIBS ${LIBS} ${Boost_LIBRARIES}) 
include_directories (SYSTEM ${Boost_INCLUDE_DIRS})
link_directories (${Boost_LIBRARY_DIRS})

