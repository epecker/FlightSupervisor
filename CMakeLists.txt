cmake_minimum_required(VERSION 3.20)

project(supervisor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Setup configured header file
configure_file("${CMAKE_SOURCE_DIR}/include/SupervisorConfig.hpp.in" "${CMAKE_SOURCE_DIR}/include/SupervisorConfig.hpp")

# Setup dependancies
include(deps/boost.cmake)
include(deps/cadmium.cmake)
include(deps/destimes.cmake)

add_executable(supervisor "${CMAKE_SOURCE_DIR}/coupled_model/main.cpp")
target_include_directories(supervisor PUBLIC ${BOOST_INCLUDE_DIR} ${CADMIUM_INCLUDE_DIR} ${DESTIMES_INCLUDE_DIR})

add_executable(td_lp_manager "${CMAKE_SOURCE_DIR}/test_drivers/td_lp_manager.cpp")
target_include_directories(td_lp_manager PUBLIC ${BOOST_INCLUDE_DIR} ${CADMIUM_INCLUDE_DIR} ${DESTIMES_INCLUDE_DIR})