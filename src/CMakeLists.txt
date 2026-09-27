add_library(lpf STATIC
	core.c
	core.h
	debris.c
	facet.c
	facet.h
	fracture.c
	fracture.h
	poly.c
	poly.h
	tasks.c
	tasks.h
	world.c
	world.h
	../include/lpf/lpf.h
)

target_include_directories(lpf
	PUBLIC ${PROJECT_SOURCE_DIR}/include
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(lpf PUBLIC box3d)
set_target_properties(lpf PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
lpf_warnings(lpf)

# Internal headers for tests and the benchmark
add_library(lpf_internal INTERFACE)
target_include_directories(lpf_internal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lpf_internal INTERFACE lpf)
