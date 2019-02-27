add_compile_definitions(IBM=1)
#add_compile_definitions(SHRDMEM_NAME = "XcommsSharedMemory")

include_directories(
        "../X-Plane SDK/CHeaders/Widgets"
        "../X-Plane SDK/CHeaders/XPLM"
        "../X-Plane SDK/CHeaders/Wrappers"
)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -llibboost_chrono -llibboost_date_time -llibboost_system -llibboost_thread")
set(SOURCE_FILES Source/main.cpp Source/Guidance.cpp ../IPC/Source/IPC.cpp)

link_directories("${CMAKE_BINARY_DIR}/../X-Plane SDK/Libraries/Win" "C:/Program Files/MinGW/lib")

add_library(XplnComms MODULE ${SOURCE_FILES})
set_target_properties(
        XplnComms
        PROPERTIES
            PREFIX ""
            OUTPUT_NAME "win"
            SUFFIX ".xpl"
            LIBRARY_OUTPUT_DIRECTORY "E:/XPlane11/Resources/plugins/MyTest/64"
)

target_link_libraries(XplnComms
        XPLM_64.lib
        XPWidgets_64.lib
        OpenGL32.lib
        IPC.dll
        libboost_chrono-mgw81-mt-x64-1_69.a
        libboost_date_time-mgw81-mt-x64-1_69.a
        libboost_system-mgw81-mt-x64-1_69.a
        libboost_thread-mgw81-mt-x64-1_69.a
)

