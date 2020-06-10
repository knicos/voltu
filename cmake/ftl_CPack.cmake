# use build date as patch version
string(TIMESTAMP BUILD_TIME "%Y%m%d")
set(CPACK_PACKAGE_VERSION_PATCH "${BUILD_TIME}")

set(CPACK_DEBIAN_PACKAGE_MAINTAINER "UTU Future Tech Lab")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS_POLICY ">=")
set(CPACK_DEB_PACKAGE_COMPONENT ON)
set(CPACK_DEBIAN_PACKAGE_SECTION "Miscellaneous")

macro(deb_append_dependency DEPENDS)
	if ("${CPACK_DEBIAN_PACKAGE_DEPENDS}" STREQUAL "")
		set(CPACK_DEBIAN_PACKAGE_DEPENDS "${DEPENDS}")
	else()
		set(CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS}, ${DEPENDS}")
	endif()
endmacro()

if (HAVE_PYLON)
	deb_append_dependency("pylon (>= 6.1.1)")
	set(ENV{LD_LIBRARY_PATH} "=/opt/pylon/lib/")
endif()

if(WIN32)
	message(INFO "Copying DLLs: OpenCV")
	file(GLOB WINDOWS_LIBS "${OpenCV_INSTALL_PATH}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/*.dll")
	install(FILES ${WINDOWS_LIBS} DESTINATION bin)
	set(CPACK_GENERATOR "WiX")
endif()

include(CPack)
