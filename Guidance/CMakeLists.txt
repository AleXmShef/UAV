include_directories(
        "../XplnComms/Source"
)

set(SOURCE_FILES Source/main.cpp Source/PID.cpp "Source/../../XplnComms/Source/IPC.cpp")

add_executable(Guidance ${SOURCE_FILES})