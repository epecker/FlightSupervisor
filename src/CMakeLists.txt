add_executable(supervisor "supervisor.cpp" "${MavNRC_GEO}" "${SHARED_MEM}" "${RUDP}")
target_link_libraries(supervisor ${Boost_LIBRARIES} -lrt)
target_include_directories(supervisor PUBLIC ${includes_list})
target_compile_definitions(supervisor PUBLIC RT_LINUX RT_DEVS)
