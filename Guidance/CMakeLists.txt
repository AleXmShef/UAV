include_directories(
        "../XplnComms/Source"
)

set(SOURCE_FILES Source/main.cpp Source/PID.cpp Source/Guidance.cpp Source/Guidance.h Source/PIDPipeline.cpp Source/PIDPipeline.h Source/Loggable.cpp Source/Loggable.h Source/Logger.cpp Source/Logger.h Source/Waypoint.cpp Source/Waypoint.h Source/Route.cpp Source/Route.h)

add_executable(Guidance ${SOURCE_FILES})

target_link_libraries(Guidance
        IPC.dll
)