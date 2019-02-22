add_compile_definitions(IBM=1)

include_directories("../X-Plane SDK/CHeaders/Widgets"
                    "../X-Plane SDK/CHeaders/XPLM"
                    "../X-Plane SDK/CHeaders/Wrappers"
                    )

set(SOURCE_FILES Source/Example.cpp)

add_library(Test ${SOURCE_FILES})
set_target_properties(
        Test
        PROPERTIES
            PREFIX ""
            OUTPUT_NAME "Win"
            SUFFIX ".xpl"
)

link_libraries(Test "../X-Plane SDK/Libraries/Win/XPLM_64.lib"
                            "../X-Plane SDK/Libraries/Win/XPWidgets_64.lib"
                        )

