#--------------------------------------------------------------

cmake_minimum_required(VERSION 2.8)
project(my_example)
# List of demos with no additional dependencies 
set(POST_DEMOS
	my_example
	)


## Adding path(?)
LIST(APPEND CMAKE_PREFIX_PATH "/usr/local/completeChrono/chrono-6.0.0/install_gcc_11/lib/cmake")

find_package(Chrono
             COMPONENTS Postprocess Cascade Irrlicht
             CONFIG)

include_directories(${CHRONO_INCLUDE_DIRS})

#--------------------------------------------------------------

message(STATUS "Demo programs for POSTPROCESS module...")

foreach(PROGRAM ${POST_DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")
    set_target_properties(${PROGRAM} PROPERTIES 
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    target_link_libraries(${PROGRAM} ${CHRONO_LIBRARIES})
#    add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_postprocess)
#    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
