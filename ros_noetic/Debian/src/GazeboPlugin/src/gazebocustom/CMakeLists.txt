#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME custom)

include_directories(${PROJECT_SOURCE_DIR}/${SUB_PROJECT_NAME}/include/)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/msgproto
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/export/include/
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/lib)

file(GLOB SOURCE_FILES
gazebocustommsghandler/*.cpp
CustomMsgDispatcher.cpp
)

add_library(GazeboCoSimCustom SHARED ${SOURCE_FILES})

target_link_libraries(GazeboCoSimCustom
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
)
