set(SOURCE_FILES Source/IPC.cpp)

add_library(IPC MODULE ${SOURCE_FILES})

set_target_properties(
        IPC
        PROPERTIES
        PREFIX ""
        OUTPUT_NAME "IPC"
)

add_library(IPCXPLN MODULE ${SOURCE_FILES})

set_target_properties(
        IPCXPLN
        PROPERTIES
        PREFIX ""
        OUTPUT_NAME "IPC"
        LIBRARY_OUTPUT_DIRECTORY "E:/XPlane11/Resources/plugins/MyTest/64"
)