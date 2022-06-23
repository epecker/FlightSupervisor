cmake_minimum_required(VERSION 3.20)

project(supervisor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Setup configured header file
configure_file("${CMAKE_SOURCE_DIR}/include/SupervisorConfig.hpp.in" "${CMAKE_SOURCE_DIR}/include/SupervisorConfig.hpp")

# Cadmium requires this to be set to run in parallel
add_compile_definitions(CADMIUM_EXECUTE_CONCURRENT)

#RT-Cadmium Definitions needed in order to use the real-time clock.
add_compile_definitions(RT_LINUX)
add_compile_definitions(RT_DEVS)
#The value of this definition can be changed to anything other than -1.
add_compile_definitions(MISSED_DEADLINE_TOLERANCE=-10)

# These definitions are required to use Boost thread pools
add_compile_definitions(BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION)
add_compile_definitions(BOOST_THREAD_PROVIDES_EXECUTORS)
add_compile_definitions(BOOST_THREAD_USES_MOVE)


# Setup dependancies
include(deps/cadmium.cmake)
include(deps/rt-cadmium.cmake)
include(deps/destimes.cmake)

configure_file("${CADMIUM_INCLUDE_DIR}/cadmium/basic_model/pdevs/iestream.hpp" "${RT_CADMIUM_INCLUDE_DIR}/cadmium/basic_model/pdevs/iestream.hpp" COPYONLY) # Used to copy a missing file to rt cadmium

# Setup boost
set(Boost_USE_STATIC_LIBS on)
find_package(
	Boost 1.65 REQUIRED
	COMPONENTS system thread regex
)

# Setup includes
set(includes_list ${Boost_INCLUDE_DIRS} ${CADMIUM_INCLUDE_DIR} ${DESTIMES_INCLUDE_DIR} "${CMAKE_SOURCE_DIR}/include")
set(rt_includes_list ${Boost_INCLUDE_DIRS} ${RT_CADMIUM_INCLUDE_DIR} ${DESTIMES_INCLUDE_DIR} "${CMAKE_SOURCE_DIR}/include")

# Create all tests
add_executable(supervisor					"${CMAKE_SOURCE_DIR}/src/main.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(rt-supervisor				"${CMAKE_SOURCE_DIR}/src/rt_supervisor.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_handover_control			"${CMAKE_SOURCE_DIR}/test/drivers/td_handover_control.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_landing_routine			"${CMAKE_SOURCE_DIR}/test/drivers/td_landing_routine.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_lp_manager				"${CMAKE_SOURCE_DIR}/test/drivers/td_lp_manager.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_reposition_timer			"${CMAKE_SOURCE_DIR}/test/drivers/td_reposition_timer.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_stabilize					"${CMAKE_SOURCE_DIR}/test/drivers/td_stabilize.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_command_reposition		"${CMAKE_SOURCE_DIR}/test/drivers/td_command_reposition.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_supervisor				"${CMAKE_SOURCE_DIR}/test/drivers/td_supervisor.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_lp_reposition				"${CMAKE_SOURCE_DIR}/test/drivers/td_lp_reposition.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_blocking_input			"${CMAKE_SOURCE_DIR}/test/drivers/td_blocking_input.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_command_line_input		"${CMAKE_SOURCE_DIR}/test/drivers/td_command_line_input.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_supervisor_command_line	"${CMAKE_SOURCE_DIR}/test/drivers/td_supervisor_command_line.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_udp_output            	"${CMAKE_SOURCE_DIR}/test/drivers/td_udp_output.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_udp_input             	"${CMAKE_SOURCE_DIR}/test/drivers/td_udp_input.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_udp_input_async			"${CMAKE_SOURCE_DIR}/test/drivers/td_udp_input_async.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_shared_memory_output		"${CMAKE_SOURCE_DIR}/test/drivers/td_shared_memory_output.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")
add_executable(td_shared_memory_input		"${CMAKE_SOURCE_DIR}/test/drivers/td_shared_memory_input.cpp" "${CMAKE_SOURCE_DIR}/src/mavNRC/geo.cpp")

target_include_directories(supervisor				    PUBLIC ${includes_list})
target_include_directories(rt-supervisor			    PUBLIC ${rt_includes_list})
target_include_directories(td_handover_control		    PUBLIC ${includes_list})
target_include_directories(td_landing_routine		    PUBLIC ${includes_list})
target_include_directories(td_lp_manager			    PUBLIC ${includes_list})
target_include_directories(td_reposition_timer		    PUBLIC ${includes_list})
target_include_directories(td_stabilize				    PUBLIC ${includes_list})
target_include_directories(td_command_reposition	    PUBLIC ${includes_list})
target_include_directories(td_supervisor			    PUBLIC ${includes_list})
target_include_directories(td_lp_reposition			    PUBLIC ${includes_list})
target_include_directories(td_blocking_input		    PUBLIC ${rt_includes_list})
target_include_directories(td_command_line_input	    PUBLIC ${rt_includes_list})
target_include_directories(td_supervisor_command_line   PUBLIC ${rt_includes_list})
target_include_directories(td_udp_output                PUBLIC ${rt_includes_list})
target_include_directories(td_udp_input                 PUBLIC ${rt_includes_list})
target_include_directories(td_udp_input_async			PUBLIC ${rt_includes_list})
target_include_directories(td_shared_memory_output		PUBLIC ${rt_includes_list})
target_include_directories(td_shared_memory_input		PUBLIC ${rt_includes_list})

target_link_libraries(supervisor			            ${Boost_LIBRARIES})
target_link_libraries(rt-supervisor			            ${Boost_LIBRARIES} -lrt)
target_link_libraries(td_handover_control	            ${Boost_LIBRARIES})
target_link_libraries(td_landing_routine	            ${Boost_LIBRARIES})
target_link_libraries(td_lp_manager			            ${Boost_LIBRARIES})
target_link_libraries(td_reposition_timer	            ${Boost_LIBRARIES})
target_link_libraries(td_stabilize			            ${Boost_LIBRARIES})
target_link_libraries(td_command_reposition             ${Boost_LIBRARIES})
target_link_libraries(td_supervisor			            ${Boost_LIBRARIES})
target_link_libraries(td_lp_reposition		            ${Boost_LIBRARIES})
target_link_libraries(td_blocking_input 	            ${Boost_LIBRARIES})
target_link_libraries(td_command_line_input             ${Boost_LIBRARIES})
target_link_libraries(td_supervisor_command_line        ${Boost_LIBRARIES})
target_link_libraries(td_udp_output                     ${Boost_LIBRARIES})
target_link_libraries(td_udp_input                      ${Boost_LIBRARIES})
target_link_libraries(td_udp_input_async				${Boost_LIBRARIES})
target_link_libraries(td_shared_memory_output			${Boost_LIBRARIES} -lrt)
target_link_libraries(td_shared_memory_input			${Boost_LIBRARIES} -lrt)
