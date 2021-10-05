cmake_minimum_required(VERSION 3.20)

project(supervisor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Setup dependancies
include(deps/boost.cmake)
include(deps/cadmium.cmake)

# add_executable(supervisor "${CMAKE_SOURCE_DIR}/coupled_model/main.cpp")
# target_include_directories(supervisor PUBLIC "${CMAKE_SOURCE_DIR}/atomic_models" ${BOOST_INCLUDE_DIR} ${CADMIUM_INCLUDE_DIR})
