include_directories(
        "../XplnComms/Source"
)

set(SOURCE_FILES Source/main.cpp Source/PID.cpp Source/Guidance.cpp Source/Guidance.h Source/PIDPipeline.cpp Source/PIDPipeline.h)

add_executable(Guidance ${SOURCE_FILES})

target_link_libraries(Guidance
        IPC.dll
)