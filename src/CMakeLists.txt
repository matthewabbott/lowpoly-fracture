add_library(lpf_core OBJECT
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

target_include_directories(lpf_core
	PUBLIC ${PROJECT_SOURCE_DIR}/include
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
find_package(Threads REQUIRED) # tasks.c uses pthreads off Windows
target_link_libraries(lpf_core PUBLIC Threads::Threads)
set_target_properties(lpf_core PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
lpf_warnings(lpf_core)

# The physics backend is the only source that sees Box3D's headers: the core above never links Box3D, so a Box3D
# include anywhere but phys_box3d.c fails to compile (milestone 8's seam), and lpf passes Box3D on for linking only
add_library(lpf_box3d OBJECT phys_box3d.c)
target_include_directories(lpf_box3d PRIVATE ${PROJECT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lpf_box3d PRIVATE box3d)
set_target_properties(lpf_box3d PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON)
lpf_warnings(lpf_box3d)

add_library(lpf STATIC $<TARGET_OBJECTS:lpf_core> $<TARGET_OBJECTS:lpf_box3d>)
target_include_directories(lpf PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lpf PRIVATE box3d PUBLIC Threads::Threads)
