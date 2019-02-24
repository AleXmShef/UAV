#add_compile_definitions(IBM=1 SHRDMEM_NAME="XcommsSharedMemory")
#add_compile_definitions(SHRDMEM_NAME = "XcommsSharedMemory")

#include_directories(
#        "../X-Plane SDK/CHeaders/Widgets"
#        "../X-Plane SDK/CHeaders/XPLM"
#        "../X-Plane SDK/CHeaders/Wrappers"
#)

set(SOURCE_FILES Source/main.cpp)

add_executable(Guidance ${SOURCE_FILES})