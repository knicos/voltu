add_library(ftlctrl
	src/master.cpp
)

target_include_directories(ftlctrl PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlctrl ftlcommon ftlnet Eigen3::Eigen)

#install(TARGETS ftlctrl EXPORT ftlctrl-config
#	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
#install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT development)

#ADD_SUBDIRECTORY(test)
