cmake_minimum_required(VERSION 3.20)

project(supervisor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Setup configured header file
configure_file("${CMAKE_SOURCE_DIR}/include/SupervisorConfig.hpp.in" "${CMAKE_SOURCE_DIR}/include/SupervisorConfig.hpp")

# Cadmium requires this to be set to run in parallel
add_compile_definitions(CADMIUM_EXECUTE_CONCURRENT)

# These definitions are required to use Boost thread pools
add_compile_definitions(BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION)
add_compile_definitions(BOOST_THREAD_PROVIDES_EXECUTORS)
add_compile_definitions(BOOST_THREAD_USES_MOVE)


# Setup dependancies
include(deps/cadmium.cmake)
include(deps/destimes.cmake)

# Setup boost
set(Boost_USE_STATIC_LIBS on)
find_package(
	Boost 1.65 REQUIRED
	COMPONENTS system thread
)

# Setup includes
set(includes_list ${Boost_INCLUDE_DIRS} ${CADMIUM_INCLUDE_DIR} ${DESTIMES_INCLUDE_DIR})

# Create all tests
add_executable(supervisor				"${CMAKE_SOURCE_DIR}/src/coupled_models/main.cpp")
add_executable(td_handover_control		"${CMAKE_SOURCE_DIR}/test/drivers/td_handover_control.cpp")
add_executable(td_landing_routine		"${CMAKE_SOURCE_DIR}/test/drivers/td_landing_routine.cpp")
add_executable(td_lp_manager			"${CMAKE_SOURCE_DIR}/test/drivers/td_lp_manager.cpp")
add_executable(td_reposition_timer		"${CMAKE_SOURCE_DIR}/test/drivers/td_reposition_timer.cpp")
add_executable(td_stabilize				"${CMAKE_SOURCE_DIR}/test/drivers/td_stabilize.cpp")
add_executable(td_command_reposition	"${CMAKE_SOURCE_DIR}/test/drivers/td_command_reposition.cpp")
add_executable(td_supervisor			"${CMAKE_SOURCE_DIR}/test/drivers/td_supervisor.cpp")
add_executable(td_lp_reposition			"${CMAKE_SOURCE_DIR}/test/drivers/td_lp_reposition.cpp")

target_include_directories(supervisor				PUBLIC ${includes_list})
target_include_directories(td_handover_control		PUBLIC ${includes_list})
target_include_directories(td_landing_routine		PUBLIC ${includes_list})
target_include_directories(td_lp_manager			PUBLIC ${includes_list})
target_include_directories(td_reposition_timer		PUBLIC ${includes_list})
target_include_directories(td_stabilize				PUBLIC ${includes_list})
target_include_directories(td_command_reposition	PUBLIC ${includes_list})
target_include_directories(td_supervisor			PUBLIC ${includes_list})
target_include_directories(td_lp_reposition			PUBLIC ${includes_list})

target_link_libraries(supervisor			${Boost_LIBRARIES})
target_link_libraries(td_handover_control	${Boost_LIBRARIES})
target_link_libraries(td_landing_routine	${Boost_LIBRARIES})
target_link_libraries(td_lp_manager			${Boost_LIBRARIES})
target_link_libraries(td_reposition_timer	${Boost_LIBRARIES})
target_link_libraries(td_stabilize			${Boost_LIBRARIES})
target_link_libraries(td_command_reposition ${Boost_LIBRARIES})
target_link_libraries(td_supervisor			${Boost_LIBRARIES})
target_link_libraries(td_lp_reposition		${Boost_LIBRARIES})
