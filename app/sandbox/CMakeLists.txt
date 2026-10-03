# lowpoly-fracture sandbox: sokol (D3D11) + Dear ImGui

set(LPF_IMGUI_DIR "${PROJECT_SOURCE_DIR}/extern/imgui")
add_library(lpf_imgui STATIC
	${LPF_IMGUI_DIR}/imgui.cpp
	${LPF_IMGUI_DIR}/imgui_draw.cpp
	${LPF_IMGUI_DIR}/imgui_tables.cpp
	${LPF_IMGUI_DIR}/imgui_widgets.cpp
)
target_include_directories(lpf_imgui SYSTEM PUBLIC ${LPF_IMGUI_DIR})
set_target_properties(lpf_imgui PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON)

set(LPF_SANDBOX_FILES
	drive.cpp
	drive.h
	main.cpp
	math3d.h
	renderer.cpp
	renderer.h
	png.cpp
	png.h
	sokol_impl.c
	imgui_impl.cpp
	shaders/scene.glsl
	shaders/generated/scene.glsl.h
)

add_executable(sandbox ${LPF_SANDBOX_FILES})
target_include_directories(sandbox PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(sandbox SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/extern/sokol)
target_link_libraries(sandbox PRIVATE lpf lpf_scenes lpf_net lpf_imgui d3d11 dxgi dxguid user32 gdi32 shell32 ole32)
target_compile_definitions(sandbox PRIVATE SOKOL_D3D11 NOMINMAX _CRT_SECURE_NO_WARNINGS)
set_target_properties(sandbox PROPERTIES
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED ON
	C_STANDARD 17
	VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)
set_source_files_properties(shaders/scene.glsl PROPERTIES HEADER_FILE_ONLY TRUE)
if(MSVC)
	# Windowed app that keeps main() and a console for automation output
	target_link_options(sandbox PRIVATE /SUBSYSTEM:CONSOLE)
endif()
