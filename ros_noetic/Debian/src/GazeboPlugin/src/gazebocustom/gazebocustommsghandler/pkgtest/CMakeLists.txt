#/* Copyright 2019 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME pkgtest)

find_package(GTest REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/msgproto
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/export/include/
${GTEST_INCLUDE_DIRS}
${GMOCK_INCLUDE_DIRS}
)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/bin )

ADD_EXECUTABLE( InitSubscribeCustomMsgHandlerTest InitSubscribeCustomMsgHandlerTest.cpp TestPoseMsgHandler.cpp ) 

TARGET_LINK_LIBRARIES( InitSubscribeCustomMsgHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimCustom.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( publishCustomMessageHandlerTest publishCustomMessageHandlerTest.cpp TestPoseMsgHandler.cpp ) 

TARGET_LINK_LIBRARIES( publishCustomMessageHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimCustom.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( subscribeCustomMessageHandlerTest subscribeCustomMessageHandlerTest.cpp TestPoseMsgHandler.cpp ) 

TARGET_LINK_LIBRARIES( subscribeCustomMessageHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimCustom.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


