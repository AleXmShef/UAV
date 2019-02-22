add_compile_definitions(IBM=1)

include_directories(
        "../X-Plane SDK/CHeaders/Widgets"
        "../X-Plane SDK/CHeaders/XPLM"
        "../X-Plane SDK/CHeaders/Wrappers"
)

set(SOURCE_FILES Source/Example.cpp)

link_directories("${CMAKE_BINARY_DIR}/../X-Plane SDK/Libraries/Win")

add_library(XplnComms SHARED ${SOURCE_FILES})
set_target_properties(
        XplnComms
        PROPERTIES
            PREFIX ""
            OUTPUT_NAME "Win"
            SUFFIX ".xpl"
)

target_link_libraries(
        XplnComms XPLM_64.lib
        XPWidgets_64.lib
        OpenGL32.lib
)

