
### Lib ELAS ###################################################################

# directories
set (LIBELAS_SRC_DIR elas)
# sources
FILE(GLOB LIBELAS_SRC_FILES "elas/*.cpp")

add_library(libelas ${LIBELAS_SRC_FILES})
target_include_directories(libelas
	PRIVATE elas)


