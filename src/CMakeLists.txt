add_executable(supervisor "supervisor.cpp")

if (UNIX AND NOT APPLE)
    target_link_libraries(supervisor -lrt)
endif ()

target_sources(supervisor PRIVATE "${MavNRC_GEO}" "${SHARED_MEM}" "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(supervisor ${Boost_LIBRARIES} ${rudp_LIBRARY})
target_include_directories(supervisor PUBLIC ${includes_list})
target_compile_definitions(supervisor PUBLIC RT_LINUX RT_DEVS)
