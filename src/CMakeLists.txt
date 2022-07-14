add_executable(supervisor		"main.cpp" 			"${MavNRC_GEO}")
add_executable(rt-supervisor	"rt_supervisor.cpp" "${MavNRC_GEO}" "${SHARED_MEM}")

target_include_directories(supervisor		PUBLIC ${includes_list})
target_include_directories(rt-supervisor	PUBLIC ${includes_list})

target_compile_definitions(rt-supervisor PUBLIC RT_LINUX RT_DEVS)

target_link_libraries(supervisor	${Boost_LIBRARIES})
target_link_libraries(rt-supervisor	${Boost_LIBRARIES} -lrt)