#/* Copyright 2019-2021 The MathWorks, Inc. */
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

ADD_EXECUTABLE( imuHandlerTest imuHandlerTest.cpp  )

TARGET_LINK_LIBRARIES( imuHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( imageHandlerTest imageHandlerTest.cpp )

TARGET_LINK_LIBRARIES( imageHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )



ADD_EXECUTABLE( laserHandlerTest laserHandlerTest.cpp )

TARGET_LINK_LIBRARIES( laserHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getTopicHandlerTest getTopicHandlerTest.cpp )

TARGET_LINK_LIBRARIES( getTopicHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( maxStepSizeHandlerTest maxStepSizeHandlerTest.cpp )

TARGET_LINK_LIBRARIES( maxStepSizeHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( applyJointTorqueHandlerTest applyJointTorqueHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( applyJointTorqueHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( applyLinkWrenchHandlerTest applyLinkWrenchHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( applyLinkWrenchHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getPoseHandlerTest getPoseHandlerTest.cpp  )  

TARGET_LINK_LIBRARIES( getPoseHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getModelInfoHandlerTest getModelInfoHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( getModelInfoHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( setJointPositionHandlerTest setJointPositionHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( setJointPositionHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( setJointVelocityHandlerTest setJointVelocityHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( setJointVelocityHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( setLinkWorldPoseHandlerTest setLinkWorldPoseHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( setLinkWorldPoseHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( setLinkLinearVelocityHandlerTest setLinkLinearVelocityHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( setLinkLinearVelocityHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( setLinkAngularVelocityHandlerTest setLinkAngularVelocityHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( setLinkAngularVelocityHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getJointStateHandlerTest getJointStateHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( getJointStateHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getLinkStateHandlerTest getLinkStateHandlerTest.cpp ) 

TARGET_LINK_LIBRARIES( getLinkStateHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( setGazeboModelParamHandlerTest setGazeboModelParamHandlerTest.cpp testUtils.cpp ) 

TARGET_LINK_LIBRARIES( setGazeboModelParamHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( setGazeboModelLinkParamHandlerTest setGazeboModelLinkParamHandlerTest.cpp testUtils.cpp ) 

TARGET_LINK_LIBRARIES( setGazeboModelLinkParamHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( setGazeboModelJointParamHandlerTest setGazeboModelJointParamHandlerTest.cpp testUtils.cpp ) 

TARGET_LINK_LIBRARIES( setGazeboModelJointParamHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getGazeboModelParamHandlerTest getGazeboModelParamHandlerTest.cpp testUtils.cpp ) 

TARGET_LINK_LIBRARIES( getGazeboModelParamHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( getGazeboModelSDFHandlerTest getGazeboModelSDFHandlerTest.cpp  testUtils.cpp ) 

TARGET_LINK_LIBRARIES( getGazeboModelSDFHandlerTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

