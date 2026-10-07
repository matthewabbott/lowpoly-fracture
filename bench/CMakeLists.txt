add_executable(lpf_bench main.c fractures.c fractures.h pair.c pair.h twin.c twin.h)
target_link_libraries(lpf_bench PRIVATE lpf lpf_scenes lpf_net)
target_include_directories(lpf_bench PRIVATE ${PROJECT_SOURCE_DIR}/src) # fractures.c reaches the fracture job, as the tests do
set_target_properties(lpf_bench PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
target_compile_definitions(lpf_bench PRIVATE _CRT_SECURE_NO_WARNINGS)
lpf_warnings(lpf_bench)
