add_library(lpf STATIC
	core.c
	core.h
	debris.c
	facet.c
	facet.h
	fracture.c
	fracture.h
	gait.c
	impact.c
	link.c
	lpmath.c
	poly.c
	phys.h
	phys_box3d.c
	poly.h
	rig.c
	solve.c
	solve.h
	split.c
	step.c
	stress.c
	supply.c
	tasks.c
	tasks.h
	wheel.c
	world.c
	world.h
	../include/lpf/lpf.h
	../include/lpf/lpmath.h
)

target_include_directories(lpf
	PUBLIC ${PROJECT_SOURCE_DIR}/include
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
find_package(Threads REQUIRED) # tasks.c uses pthreads off Windows
target_link_libraries(lpf PUBLIC box3d Threads::Threads)
set_target_properties(lpf PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
lpf_warnings(lpf)
