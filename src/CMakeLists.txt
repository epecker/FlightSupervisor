add_executable(supervisor "supervisor.cpp" "${MavNRC_GEO}" "${SHARED_MEM}" "${RUDP}")

if (APPLE)
    target_link_libraries(supervisor ${Boost_LIBRARIES})
else(UNIX)
    target_link_libraries(supervisor ${Boost_LIBRARIES} -lrt)
endif ()

target_include_directories(supervisor PUBLIC ${includes_list})
target_compile_definitions(supervisor PUBLIC RT_LINUX RT_DEVS)
