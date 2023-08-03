#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME transport)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/export/include/
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/lib)

file(GLOB SOURCE_FILES
Client.cpp
Connection.cpp 
ConnectionManager.cpp
readWithTimeout.cpp
Server.cpp
)

add_library(GazeboCoSimTransport SHARED ${SOURCE_FILES})

target_link_libraries(GazeboCoSimTransport
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
)
