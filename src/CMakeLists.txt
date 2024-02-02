set(TARGET_NAME VPCR)

file(GLOB ${TARGET_NAME}_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")


add_executable(${TARGET_NAME} ${${TARGET_NAME}_SOURCES})
target_link_libraries(${TARGET_NAME} PUBLIC tga_vulkan tga_utils CLI tinygltf GUI ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(${TARGET_NAME} tga_shaders)
if(WIN32)
    set_property(TARGET ${TARGET_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${EXAMPLES_WORKING_DIR}")
endif(WIN32)

