add_library(lpf_scenes STATIC scenes.c scenes.h script.c script.h dump.c dump.h)
target_include_directories(lpf_scenes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lpf_scenes PUBLIC lpf)
target_compile_definitions(lpf_scenes PRIVATE _CRT_SECURE_NO_WARNINGS) # fopen and sscanf for replay scripts
set_target_properties(lpf_scenes PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
lpf_warnings(lpf_scenes)
