# The dual-dialect toy (DESIGN.md), from the kernels and twins that `lab.py gen` (gen_toy.py) wrote to gen/toy: per
# dialect F, V4 and D, the helper battery (battery_<d>) and the twin driver (toy_<d>). Included by the lab's
# CMakeLists.txt when gen/toy exists; reuses its lab_no_contract, lab_quiet and vk_util. Run the binaries from lab/gpu.
set(TOY_SRC "${CMAKE_CURRENT_SOURCE_DIR}")
set(TOY_GEN "${LAB_GEN}/toy")

# A twin object library: the glue includes the generated C++ named by TWIN_GEN (a wrapper file carries the define, as
# the lab's lab_twin does).
function(toy_twin name glue generated)
	set(wrapper "${CMAKE_BINARY_DIR}/twins/toy_${name}.cpp")
	file(WRITE "${wrapper}.tmp" "#define TWIN_GEN \"${generated}\"\n#include \"${TOY_SRC}/${glue}\"\n")
	file(COPY_FILE "${wrapper}.tmp" "${wrapper}" ONLY_IF_DIFFERENT)
	add_library(toytwin_${name} OBJECT "${wrapper}")
	target_include_directories(toytwin_${name} PRIVATE "${TOY_SRC}" "${LAB_SRC}")
	set_source_files_properties("${wrapper}" PROPERTIES OBJECT_DEPENDS "${generated};${TOY_SRC}/${glue}")
	lab_quiet(toytwin_${name})
	lab_no_contract(toytwin_${name})
	toy_fp_traps(toytwin_${name})
endfunction()

# The floating-point sentinel reads the exception flags the kernels' code raises, so the twins are compiled without
# floating-point speculation: clang's default (-fno-trapping-math) may evaluate a guarded operation ahead of its guard,
# or in an unused vector lane, and raise a flag the source never raises. maytrap forbids introducing exceptions and
# changes no value (gcc's default -ftrapping-math is the same); MSVC's /fp:precise has not shown it.
function(toy_fp_traps target)
	if(LAB_MSVC_FRONTEND AND NOT CMAKE_C_COMPILER_ID STREQUAL "MSVC")
		target_compile_options(${target} PRIVATE /clang:-ffp-exception-behavior=maytrap)
	elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		target_compile_options(${target} PRIVATE -ffp-exception-behavior=maytrap)
	elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		target_compile_options(${target} PRIVATE -ftrapping-math)
	endif()
endfunction()

function(toy_c17 target)
	set_property(TARGET ${target} PROPERTY C_STANDARD 17)
	target_include_directories(${target} PRIVATE "${TOY_SRC}")
	lab_no_contract(${target})
endfunction()

# Dialect-free C: the scene generator (double), the CPU stages (integers), the trajectory files
if(EXISTS "${TOY_SRC}/scene.c" AND EXISTS "${TOY_SRC}/stages.c")
	add_library(toy_common STATIC "${TOY_SRC}/scene.c" "${TOY_SRC}/stages.c" "${TOY_SRC}/traj.c")
	toy_c17(toy_common)
	if(NOT WIN32)
		target_link_libraries(toy_common PUBLIC m)
	endif()
endif()

# Box3D on the toy's scenes (step 4b's reference): the same scene generator and trajectory files, Box3D's solver
if(TARGET box3d AND TARGET toy_common AND EXISTS "${TOY_SRC}/b3ref2.c")
	add_executable(b3ref2 "${TOY_SRC}/b3ref2.c")
	target_link_libraries(b3ref2 PRIVATE box3d toy_common)
	toy_c17(b3ref2)
endif()

foreach(d F V4 D)
	if(EXISTS "${TOY_GEN}/${d}/battery.cpp")
		toy_twin(battery_${d} battery_glue.cpp "${TOY_GEN}/${d}/battery.cpp")
		add_executable(battery_${d} "${TOY_SRC}/battery.c" $<TARGET_OBJECTS:toytwin_battery_${d}>)
		target_compile_definitions(battery_${d} PRIVATE DIALECT_${d})
		target_link_libraries(battery_${d} PRIVATE vk_util)
		toy_c17(battery_${d})
	endif()
	if(EXISTS "${TOY_GEN}/${d}/kernels.cpp" AND TARGET toy_common)
		toy_twin(kernels_${d} kernels_glue.cpp "${TOY_GEN}/${d}/kernels.cpp")
		add_executable(toy_${d} "${TOY_SRC}/toy.c" "${TOY_SRC}/quant.c" $<TARGET_OBJECTS:toytwin_kernels_${d}>)
		target_compile_definitions(toy_${d} PRIVATE DIALECT_${d})
		target_link_libraries(toy_${d} PRIVATE toy_common vk_util)
		toy_c17(toy_${d})
		# the narrowphase's corpus (gate 3): the same twin objects, the kernels' SPIR-V on the GPUs
		if(EXISTS "${TOY_SRC}/narrow.c")
			add_executable(narrow_${d} "${TOY_SRC}/narrow.c" "${TOY_SRC}/quant.c" $<TARGET_OBJECTS:toytwin_kernels_${d}>)
			target_compile_definitions(narrow_${d} PRIVATE DIALECT_${d})
			target_link_libraries(narrow_${d} PRIVATE toy_common vk_util)
			toy_c17(narrow_${d})
		endif()
	endif()
endforeach()
