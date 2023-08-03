/* Copyright 2019-2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/Client.hpp"

#include "gazebotransport/GazeboServer.hpp"
#include "TestPoseMsgHandler.hpp"
#include "mw.internal.robotics.gazebotransport.TestMsgs.pb.h"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/InitSubscribeCustomMsgHandler.hpp"

#include "gazebotransport/Server.hpp"
#include "gazebotransport/Callback.hpp"
#include "gazebotransport/PacketEnding.hpp"

using namespace mw::internal::robotics::gazebotransport;

/// Test InitSubscribeCustomMsgHandler functionality
/// These tests are performed to validate message type for InitSubscribeCustomMsgHandler handler
class InitSubscribeCustomMsgHandlerTest : public testing::Test {
  public:
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    /// create CustomMsgDispatcher object
    std::shared_ptr<robotics::gazebotransport::CustomMsgDispatcher> m_customDispatch;
    std::string ipAddress = "127.0.0.1";

    /// Gazebo Node pointer
    gazebo::transport::NodePtr nodeSub;

    /// Threads to publish custom message
    std::shared_ptr<std::thread> th0;
    std::shared_ptr<std::thread> th1;

    int clientEnd = 1;

    /// Indicates publishing of custom message 0 and custom message 1 is started
    bool startedPub0 = false;
    bool startedPub1 = false;

    /// Publish custom message0 on a topic of gazebo
    void publishCustomMessage0() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub =
            node->Advertise<TestPose>("/gazebo/default/test_topic0");

        TestPose ground_pose0;
        ground_pose0.set_x(10);
        ground_pose0.set_y(20);
        ground_pose0.set_z(30);
        ground_pose0.set_w(40);

        /// Publish custom message in a loop
        while (this->clientEnd) {
            pub->Publish(ground_pose0);
            this->startedPub0 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }

    /// Publish custom message1 on a topic of gazebo
    void publishCustomMessage1() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub =
            node->Advertise<TestPose>("/gazebo/default/test_topic1");

        TestPose ground_pose1;
        ground_pose1.set_x(40);
        ground_pose1.set_y(30);
        ground_pose1.set_z(20);
        ground_pose1.set_w(10);

        /// Publish custom message in a loop
        while (this->clientEnd) {
            pub->Publish(ground_pose1);
            this->startedPub1 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }

    void SetUp() {
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(10000));

        /// Start Gazebo Simulator Server
        gazebo::setupServer();

        /// Launch custom message publisher threads
        th0 = std::make_shared<std::thread>(
            &InitSubscribeCustomMsgHandlerTest::publishCustomMessage0, this);
        th1 = std::make_shared<std::thread>(
            &InitSubscribeCustomMsgHandlerTest::publishCustomMessage1, this);

        /// Ensure the custom message pub is started
        while (!this->startedPub0 || !this->startedPub1) {
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }

        /// Init Node, SubPtr and GazeboMsgRepo for msgHandler
        nodeSub = gazebo::transport::NodePtr(new gazebo::transport::Node());
        nodeSub->Init();

        m_customDispatch = std::make_shared<robotics::gazebotransport::CustomMsgDispatcher>();

        /// register InitSubscribeCustomMsgHandler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::InitSubscribeCustomMsgHandler>(
                this->m_customDispatch));

        /// register test custom message handler
        m_customDispatch->registerCustomHandler(
            std::make_shared<robotics::gazebotransport::TestPoseMsgHandler>(this->nodeSub),
            "gazebo_msgs/TestPose");
    }

    void TearDown() {
        /// Close Threads
        this->clientEnd = 0;
        th0->join();
        th1->join();

        m_server->stop();
        m_client->shutdown();

        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
};

/// POSITIVE TEST
/// Test InitSubscribeCustomMsgHandler for valid message type
TEST_F(InitSubscribeCustomMsgHandlerTest, InitSubscribeCustomMsgHandlerTestValidMsgType) {

    bool m_success = false;

    // create Packet message with INIT_CUSTOM_MESSAGE_SUBSCRIBER type
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_subscriber()->set_topic_name("/gazebo/default/test_topic1");
    m_message.mutable_init_custom_subscriber()->set_message_type("gazebo_msgs/TestPose");

    // serialize message
    auto msg = m_message.SerializeAsString();

    // send packet message & receive reply message over server-client interaction
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    if (replyMsg) {
        mw::internal::robotics::gazebotransport::Packet reply;
        reply.ParseFromString(*replyMsg);
        m_success = reply.has_status() && !reply.status();
    } else {
        m_success = false;
    }

    // expected TRUE as valid message type
    EXPECT_TRUE(m_success);
}

/// NEGATIVE TEST
/// Test InitSubscribeCustomMsgHandler for invalid message type
TEST_F(InitSubscribeCustomMsgHandlerTest, InitSubscribeCustomMsgHandlerTestInValidMsgType) {

    bool m_success = false;

    // create Packet message with INIT_CUSTOM_MESSAGE_SUBSCRIBER type
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_subscriber()->set_topic_name("/gazebo/default/test_topic1");
    m_message.mutable_init_custom_subscriber()->set_message_type("gazebo_msgs/test_incorrect");

    // serialize message
    auto msg = m_message.SerializeAsString();

    // send packet message & receive reply message over server-client interaction
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    if (replyMsg) {
        mw::internal::robotics::gazebotransport::Packet reply;
        reply.ParseFromString(*replyMsg);

        m_success = reply.has_status() && !reply.status();
    } else {
        m_success = false;
    }

    // expected FALSE as invalid message type
    EXPECT_FALSE(m_success);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
