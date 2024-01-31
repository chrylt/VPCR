file(GLOB GUI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.cpp" 
				  "${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_glfw.cpp"
				  "${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_vulkan.cpp")

add_library(GUI ${GUI_SRC})
target_link_libraries(GUI tga_vulkan)

target_include_directories(GUI PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR} 
	${CMAKE_CURRENT_SOURCE_DIR}/imgui 
	${CMAKE_CURRENT_SOURCE_DIR}/backends
	${CMAKE_CURRENT_SOURCE_DIR}/imgui/examples/libs/glfw/include
	)