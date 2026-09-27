cmake_minimum_required(VERSION 3.22)

project(lowpoly_fracture
	VERSION 0.1.0
	DESCRIPTION "Low-poly polygonal destruction on Box3D"
	LANGUAGES C CXX
)

option(LPF_SANDBOX "Build the sokol sandbox app" ON)
option(LPF_TESTS "Build the unit and determinism tests" ON)
option(LPF_BENCH "Build the headless benchmark" ON)
option(LPF_WARNINGS_AS_ERRORS "Treat warnings in our own code as errors" ON)
option(LPF_SANITIZE "Build with AddressSanitizer" OFF)
option(LPF_ASSERTS "Keep lpf asserts in optimized builds (asserts never change state, so hashes are unaffected)" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Box3D links the static CRT on MSVC; everything must agree.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Determinism: no FMA contraction, no fast-math anywhere (see docs/determinism-rules.md).
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
	if(CMAKE_C_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
		add_compile_options(/clang:-ffp-contract=off)
	else()
		add_compile_options(-ffp-contract=off)
	endif()
elseif(MSVC)
	# MSVC: /fp:precise never contracts to FMA unless /fp:contract is given
	add_compile_options(/fp:precise)
endif()

if(LPF_ASSERTS)
	add_compile_definitions(LP_FORCE_ASSERT)
endif()

if(LPF_SANITIZE)
	if(MSVC)
		add_compile_options(/fsanitize=address /Zi)
		add_link_options(/INCREMENTAL:NO /DEBUG)
	endif()
endif()

# Box3D is vendored at a pinned commit (extern/box3d/PATCHES.md). Only its library is built.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.25)
	add_subdirectory(extern/box3d SYSTEM)
else()
	add_subdirectory(extern/box3d)
endif()

# Warnings for our own targets
function(lpf_warnings target)
	if(MSVC AND NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
		target_compile_options(${target} PRIVATE /W4 /wd4201)
		if(LPF_WARNINGS_AS_ERRORS)
			target_compile_options(${target} PRIVATE /WX)
		endif()
	else()
		target_compile_options(${target} PRIVATE -Wall -Wextra -Wno-missing-field-initializers -Wno-unused-parameter)
		if(CMAKE_C_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
			target_compile_options(${target} PRIVATE /W4 -Wno-language-extension-token -Wno-nonportable-system-include-path)
		endif()
		if(LPF_WARNINGS_AS_ERRORS)
			target_compile_options(${target} PRIVATE -Werror)
		endif()
	endif()
endfunction()

add_subdirectory(src)
add_subdirectory(scenes)

if(LPF_TESTS)
	enable_testing()
	add_subdirectory(test)
endif()

if(LPF_BENCH)
	add_subdirectory(bench)
endif()

if(LPF_SANDBOX AND WIN32)
	add_subdirectory(app/sandbox)
	if(MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT sandbox)
	endif()
endif()
