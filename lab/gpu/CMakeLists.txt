# The GPU lab (research code, outside the engine and its CI; README.md). Builds E11's runners from src/ and the
# kernels and twins that `lab.py gen` wrote to gen/ on Windows: probe, mulbench, harness_<variant>, rows_<variant>,
# and the positive controls harness_F-fma and rows_V1-fma (twins whose compiler may fuse a*b+c).
cmake_minimum_required(VERSION 3.22)
project(lpf_gpu_lab LANGUAGES C CXX)

find_package(Vulkan REQUIRED)
find_package(Threads REQUIRED)

set(LAB_GEN "${CMAKE_CURRENT_SOURCE_DIR}/gen" CACHE PATH "kernels and twins from lab.py gen")
if(NOT EXISTS "${LAB_GEN}/variants.cmake")
	message(FATAL_ERROR "${LAB_GEN}/variants.cmake is missing: run `python lab/gpu/lab.py gen` on Windows (or copy gen/ here)")
endif()
include("${LAB_GEN}/variants.cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
# The static C runtime, as the engine and Box3D use it (one runtime per binary)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(LAB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src")

if(WIN32)
	add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

set(LAB_MSVC_FRONTEND OFF)
if(MSVC OR CMAKE_C_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
	set(LAB_MSVC_FRONTEND ON)
endif()

# Every file is built without contraction or fast math (docs/determinism-rules.md): the scenarios are generated in
# double on the CPU and must be the same bits everywhere, and the twins are the reference.
function(lab_no_contract target)
	if(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
		target_compile_options(${target} PRIVATE /fp:precise)
	elseif(LAB_MSVC_FRONTEND)
		target_compile_options(${target} PRIVATE /clang:-ffp-contract=off)
	else()
		target_compile_options(${target} PRIVATE -ffp-contract=off)
	endif()
endfunction()

# The positive control: contraction allowed and FMA instructions available, so the harness can show it sees a change.
function(lab_contract target)
	target_compile_definitions(${target} PRIVATE LAB_FMA)
	if(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
		target_compile_options(${target} PRIVATE /fp:contract /arch:AVX2)
	elseif(LAB_MSVC_FRONTEND)
		target_compile_options(${target} PRIVATE /arch:AVX2 /clang:-ffp-contract=fast /clang:-mfma)
	else()
		target_compile_options(${target} PRIVATE -ffp-contract=fast)
		if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
			target_compile_options(${target} PRIVATE -mavx2 -mfma)
		endif()
	endif()
endfunction()

function(lab_quiet target) # the generated twins are Slang's C++, not ours
	if(LAB_MSVC_FRONTEND)
		target_compile_options(${target} PRIVATE /W0)
	else()
		target_compile_options(${target} PRIVATE -w)
	endif()
endfunction()

add_library(vk_util STATIC "${LAB_SRC}/vk_util.c")
target_include_directories(vk_util PUBLIC "${LAB_SRC}")
target_link_libraries(vk_util PUBLIC Vulkan::Vulkan Threads::Threads)
if(NOT WIN32)
	target_link_libraries(vk_util PUBLIC m)
endif()
lab_no_contract(vk_util)

# A twin object library: the glue includes the generated C++ named by TWIN_GEN; a wrapper file carries the define,
# so no quoted path has to pass through a compiler command line.
function(lab_twin name glue generated contract)
	set(wrapper "${CMAKE_BINARY_DIR}/twins/${name}.cpp")
	file(WRITE "${wrapper}.tmp" "#define TWIN_GEN \"${generated}\"\n#include \"${LAB_SRC}/${glue}\"\n")
	file(COPY_FILE "${wrapper}.tmp" "${wrapper}" ONLY_IF_DIFFERENT)
	add_library(twin_${name} OBJECT "${wrapper}")
	target_include_directories(twin_${name} PRIVATE "${LAB_SRC}")
	set_source_files_properties("${wrapper}" PROPERTIES OBJECT_DEPENDS "${generated};${LAB_SRC}/${glue}")
	lab_quiet(twin_${name})
	if(contract)
		lab_contract(twin_${name})
	else()
		lab_no_contract(twin_${name})
	endif()
endfunction()

function(lab_runner name source defs twin)
	add_executable(${name} "${LAB_SRC}/${source}" $<TARGET_OBJECTS:twin_${twin}>)
	target_compile_definitions(${name} PRIVATE ${defs})
	target_link_libraries(${name} PRIVATE vk_util)
	lab_no_contract(${name})
endfunction()

# The full solve (harness.c): one runner per variant, plus the FMA positive control on F's twin
foreach(v IN LISTS LAB_SOLVE_VARIANTS)
	lab_twin(${v} twin_glue.cpp "${LAB_GEN}/${v}/twin.cpp" OFF)
	lab_runner(harness_${v} harness.c "${LAB_SOLVE_DEFS_${v}}" ${v})
endforeach()
lab_twin(F-fma twin_glue.cpp "${LAB_GEN}/F/twin.cpp" ON)
lab_runner(harness_F-fma harness.c "VAR_F" F-fma)

# The row benchmark (rows.c), likewise
foreach(v IN LISTS LAB_ROWS_VARIANTS)
	lab_twin(rows_${v} rows_glue.cpp "${LAB_GEN}/rows/${v}.cpp" OFF)
	lab_runner(rows_${v} rows.c "${LAB_ROWS_DEF_${v}}" rows_${v})
endforeach()
lab_twin(rows_V1-fma rows_glue.cpp "${LAB_GEN}/rows/V1.cpp" ON)
lab_runner(rows_V1-fma rows.c "VAR_F" rows_V1-fma)

foreach(tool probe mulbench)
	add_executable(${tool} "${LAB_SRC}/${tool}.c")
	target_link_libraries(${tool} PRIVATE vk_util)
	lab_no_contract(${tool})
endforeach()

# Box3D (the engine's vendored copy, extern/box3d) for the reference programs: E11's b3ref (Box3D's solver cost at a
# contact count) and the toy's b3ref2. lab.py remote ships it beside the lab.
option(LAB_BOX3D "Build the Box3D reference programs" ON)
set(LAB_BOX3D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../extern/box3d" CACHE PATH "Box3D's source")
if(LAB_BOX3D AND EXISTS "${LAB_BOX3D_DIR}/CMakeLists.txt")
	add_subdirectory("${LAB_BOX3D_DIR}" box3d EXCLUDE_FROM_ALL)
	add_executable(b3ref "${LAB_SRC}/b3ref.c")
	target_link_libraries(b3ref PRIVATE box3d)
	lab_no_contract(b3ref)
endif()

# The dual-dialect toy (toy/DESIGN.md), when gen_toy.py has generated its kernels and twins
if(EXISTS "${LAB_GEN}/toy")
	add_subdirectory(toy)
endif()
