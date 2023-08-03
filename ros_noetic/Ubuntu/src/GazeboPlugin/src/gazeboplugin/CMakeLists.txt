#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME gazeboplugin)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/msgproto
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/export/include/
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/lib)

add_library(GazeboCoSimPlugin SHARED GazeboPlugin.cpp )

target_link_libraries(GazeboCoSimPlugin
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimCustom.so
)

