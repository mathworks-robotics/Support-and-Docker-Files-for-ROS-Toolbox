#/* Copyright 2019-2020 The MathWorks, Inc. */
SET(SUB_PROJECT_NAME pkgtest)

find_package(GTest REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/msgproto
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/export/include/
${PROJECT_SOURCE_DIR}/src
${GTEST_INCLUDE_DIRS}
${GMOCK_INCLUDE_DIRS}
)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/bin )

ADD_EXECUTABLE( pluginTest pluginTest.cpp )

TARGET_LINK_LIBRARIES( pluginTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

ADD_EXECUTABLE( forceTest forceTest.cpp ) 

TARGET_LINK_LIBRARIES( forceTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


ADD_EXECUTABLE( pluginTestCustomHandler pluginTestCustomHandler.cpp ) 

TARGET_LINK_LIBRARIES( pluginTestCustomHandler LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimCustom.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/export/lib )

add_library(CustomHandlerTestPlugin SHARED CustomHandlerTestPlugin.cpp )

target_link_libraries(CustomHandlerTestPlugin
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimCustom.so
)

file(GLOB SOURCE_FILES
${PROJECT_SOURCE_DIR}/src/gazebocustom/gazebocustommsghandler/pkgtest/TestPoseMsgHandler.cpp
${PROJECT_SOURCE_DIR}/src/gazebocustom/gazebocustommsghandler/pkgtest/ImageMatMsgHandler.cpp
${PROJECT_SOURCE_DIR}/src/gazebocustom/gazebocustommsghandler/pkgtest/IMUMatMsgHandler.cpp
)

add_library(GazeboTestPlugin SHARED GazeboTestPlugin.cpp  ${SOURCE_FILES} )

target_link_libraries(GazeboTestPlugin
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimCustom.so
)


ADD_EXECUTABLE( gzPluginTest gzPluginTest.cpp  ${PROJECT_SOURCE_DIR}/src/gazeboserver/gazebomsghandler/pkgtest/testUtils.cpp )

TARGET_LINK_LIBRARIES( gzPluginTest LINK_PUBLIC
${GAZEBO_LIBRARIES}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimServer.so
${PROJECT_SOURCE_DIR}/export/lib/libGazeboCoSimTransport.so
${PROJECT_SOURCE_DIR}/export/lib/libmsgproto.so
${GTEST_LIBRARIES}
${GMOCK_LIBRARIES}
pthread )

