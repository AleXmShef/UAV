include_directories(
        "../XplnComms/Source"
)

set(SOURCE_FILES Source/main.cpp Source/PID.cpp)

add_executable(Guidance ${SOURCE_FILES})

target_link_libraries(Guidance
        IPC.dll
)