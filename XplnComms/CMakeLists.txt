add_compile_definitions(IBM=1)
#add_compile_definitions(SHRDMEM_NAME = "XcommsSharedMemory")

include_directories(
        "../X-Plane SDK/CHeaders/Widgets"
        "../X-Plane SDK/CHeaders/XPLM"
        "../X-Plane SDK/CHeaders/Wrappers"
)

set(SOURCE_FILES Source/main.cpp Source/Guidance.cpp ../IPC/Source/IPC.cpp)
set(GCC_COVERAGE_COMPILE_FLAGS "-Wwrite-strings")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

link_directories("${CMAKE_BINARY_DIR}/../X-Plane SDK/Libraries/Win")

add_library(XplnComms MODULE ${SOURCE_FILES})
set_target_properties(
        XplnComms
        PROPERTIES
            PREFIX ""
            OUTPUT_NAME "win"
            SUFFIX ".xpl"
            LIBRARY_OUTPUT_DIRECTORY "E:/XPlane11/Resources/plugins/MyTest/64"
)

target_link_libraries(
        XplnComms XPLM_64.lib
        XPWidgets_64.lib
        OpenGL32.lib
        IPC.dll
)

