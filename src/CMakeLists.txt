add_executable(main							"main.cpp" 		 "${MavNRC_GEO}")
add_executable(supervisor					"supervisor.cpp" "${MavNRC_GEO}" "${SHARED_MEM}" "${RUDP}")

target_include_directories(main				PUBLIC ${includes_list})
target_include_directories(supervisor		PUBLIC ${includes_list})

target_compile_definitions(supervisor 		PUBLIC RT_LINUX RT_DEVS)

target_link_libraries(main					${Boost_LIBRARIES})
target_link_libraries(supervisor			${Boost_LIBRARIES} -lrt)