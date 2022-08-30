cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(supervisor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


############################
###  Configured Headers  ###
############################
configure_file("${CMAKE_SOURCE_DIR}/src/SupervisorConfig.hpp.in" "${CMAKE_SOURCE_DIR}/src/SupervisorConfig.hpp")

#####################################
###  Global Compiler Definitions  ###
#####################################
# Cadmium
add_compile_definitions(CADMIUM_EXECUTE_CONCURRENT) # Allows for parallel execution
add_compile_definitions(MISSED_DEADLINE_TOLERANCE=50000) # Sets the deadline tolerance in microseconds; must not be -1

# Boost
# Enables thread pools
add_compile_definitions(BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION)
add_compile_definitions(BOOST_THREAD_PROVIDES_EXECUTORS)
add_compile_definitions(BOOST_THREAD_USES_MOVE)


##########################
###  Dependency Setup  ###
##########################
include(deps/cadmium.cmake) 	# Downloads cadmium and sets up variables for includes
include(deps/destimes.cmake) 	# Downloads destimes and sets up variables for includes
include(deps/boost.cmake) 		# Searches system for Boost and sets up variables for includes


##########################
###  Global Variables  ###
##########################
set(includes_list ${Boost_INCLUDE_DIRS} ${CADMIUM_INCLUDE_DIR} ${DESTIMES_INCLUDE_DIR} "${CMAKE_SOURCE_DIR}/deps" "${CMAKE_SOURCE_DIR}/deps/RUDP/include")
set(MavNRC_GEO 	"${CMAKE_SOURCE_DIR}/deps/mavNRC/geo.cpp")
set(SHARED_MEM 	"${CMAKE_SOURCE_DIR}/deps/sharedmemorymodel/SharedMemoryModel.cpp")
set(RUDP 		"${CMAKE_SOURCE_DIR}/deps/RUDP/src/ConnectionController.cpp" "${CMAKE_SOURCE_DIR}/deps/RUDP/src/Connection.cpp")

#################################
###  Compiler Specific Fixes  ###
#################################
if (APPLE AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11.0.0"))
    configure_file("${CMAKE_SOURCE_DIR}/deps/cadmium/message_box.hpp" "${PROJECT_BINARY_DIR}/_deps/cadmium-src/include/cadmium/modeling/message_box.hpp" COPYONLY)
endif()

########################
###  Subdirectories  ###
########################
add_subdirectory(src)
add_subdirectory(test)
