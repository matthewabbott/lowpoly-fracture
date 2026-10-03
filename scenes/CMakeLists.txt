add_library(lpf_scenes STATIC scenes.c scenes.h script.c script.h dump.c dump.h)
target_include_directories(lpf_scenes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lpf_scenes PUBLIC lpf)
target_compile_definitions(lpf_scenes PRIVATE _CRT_SECURE_NO_WARNINGS) # fopen and sscanf for replay scripts
set_target_properties(lpf_scenes PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
lpf_warnings(lpf_scenes)
# The build's id in session handshakes (lpSceneDescribeSession): the git commit at configure time, so it is stale
# until the next configure (CI configures afresh)
execute_process(COMMAND git describe --always --dirty --abbrev=12 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	OUTPUT_VARIABLE lpf_build_id OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT lpf_build_id)
	set(lpf_build_id unknown)
endif()
target_compile_definitions(lpf_scenes PRIVATE LPF_BUILD_ID="${lpf_build_id}")
