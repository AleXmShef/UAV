cmake_minimum_required(VERSION 2.8)

Project(UAV CXX)

add_compile_definitions(SHRDMEM_NAME="XcommsSharedMemory" SHRDOUTPUT_NAME="XcommsOutputData" SHRDINPUT_NAME="XcommsInputData")

include_directories("C:/Program Files/boost_1_69_0" "IPC/Source")



set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  "${CMAKE_BINARY_DIR}/../Bin/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/../Bin/)

link_directories("${CMAKE_BINARY_DIR}/../Bin")

add_subdirectory(IPC)
add_subdirectory(XplnComms)
add_subdirectory(Guidance)





