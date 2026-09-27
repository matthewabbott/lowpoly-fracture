add_executable(lpf_test main.c test_macros.h test_poly.c test_fracture.c test_world.c test_debris.c)
target_link_libraries(lpf_test PRIVATE lpf_internal lpf_scenes)
set_target_properties(lpf_test PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
lpf_warnings(lpf_test)

add_test(NAME poly COMMAND lpf_test poly)
add_test(NAME fracture COMMAND lpf_test fracture)
add_test(NAME world COMMAND lpf_test world)
add_test(NAME debris COMMAND lpf_test debris)
