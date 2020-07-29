### OpenCV Codec Unit ################################################################
add_executable(mixer_unit
$<TARGET_OBJECTS:CatchTest>
	mixer_unit.cpp
)
target_include_directories(mixer_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(mixer_unit
	Threads::Threads ${OS_LIBS} ftlcommon)


add_test(MixerUnitTest mixer_unit)
