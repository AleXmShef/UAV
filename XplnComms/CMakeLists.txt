cmake_minimum_required(VERSION 2.8)

Project(XplnComms CXX)
add_compile_definitions(IBM=1)

include_directories("../XplnSDK/CHeaders/Widgets"
                    "../XplnSDK/CHeaders/XPLM"
                    "../XplnSDK/CHeaders/Wrappers"
                    )

set(SOURCE_FILES Source/Example.cpp)

add_library(Test ${SOURCE_FILES})

link_libraries(Test "../XplnSDK/Libraries/Win/XPLM_64.lib"
                            "../XplnSDK/Libraries/Win/XPWidgets_64.lib"
                        )

