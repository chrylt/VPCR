add_library(bmp INTERFACE)

target_include_directories(bmp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/multiprecision/include)
target_include_directories(bmp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/config/include)