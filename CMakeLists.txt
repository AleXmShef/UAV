cmake_minimum_required(VERSION 2.8)

Project(UAV CXX)

add_compile_definitions(SHRDMEM_NAME="XcommsSharedMemory")

include_directories("C:/Program Files/boost_1_69_0")



set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/../Bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/../Bin)

add_subdirectory(XplnComms)
add_subdirectory(Guidance)




