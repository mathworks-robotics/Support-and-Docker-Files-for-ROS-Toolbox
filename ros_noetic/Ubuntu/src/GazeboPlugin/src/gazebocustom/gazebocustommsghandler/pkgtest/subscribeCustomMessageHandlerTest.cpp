/* Copyright 2019-2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"

#include "gazebotransport/gazebocustom/gazebocustommsghandler/SubscribeCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/InitSubscribeCustomMsgHandler.hpp"
#include "TestPoseMsgHandler.hpp"
#include "mw.internal.robotics.gazebotransport.TestMsgs.pb.h"

#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "google/protobuf/util/message_differencer.h"
#include <gazebo/physics/physics.hh>

using namespace mw::internal::robotics::gazebotransport;

typedef const boost::shared_ptr<const TestPose> TestPosePtr;

class customMessageSubscribe : public testing::Test {

  public:
    std::string ipAddress = "127.0.0.1";

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /// Threads to publish custom message
    std::shared_ptr<std::thread> th0;
    std::shared_ptr<std::thread> th1;

    /// Session Time-out value
    int time_out = 5000;

    // Gazebo Node pointer
    gazebo::transport::NodePtr nodeSub;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_world;

    // Custom message dispatcher instance
    std::shared_ptr<robotics::gazebotransport::CustomMsgDispatcher> m_customDispatch;

    // Custom message storage instance
    // robotics::gazebotransport::CustomMessageContainer m_customContainer;

    /// Keep publisher threads (th0 & th1 ) alive
    int clientEnd = 1;

    // Indicates publishing of custom message 0 and custom message 1 is started
    bool startedPub0 = false;
    bool startedPub1 = false;

    // Publish custom message0 on a topic of gazebo
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

    // Publish custom message1 on a topic of gazebo
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

        /// Launch & Run GazeboServer
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();

        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(time_out));

        // Start Gazebo Simulator Server
        gazebo::setupServer();

        /// Load unit-box world file
        this->m_world = gazebo::loadWorld("world/unitBoxHandlerTest.world");
        ASSERT_TRUE(m_world != nullptr) << "Failed to load world";
        this->m_world->SetPaused(true);

        /// Launch custom message publisher threads
        th0 = std::make_shared<std::thread>(&customMessageSubscribe::publishCustomMessage0, this);
        th1 = std::make_shared<std::thread>(&customMessageSubscribe::publishCustomMessage1, this);

        // Ensure the custom message pub is started
        while (!this->startedPub0 || !this->startedPub1) {
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }

        /// Init Node, SubPtr and GazeboMsgRepo for msgHandler
        nodeSub = gazebo::transport::NodePtr(new gazebo::transport::Node());
        nodeSub->Init();

        /// Message handler starts
        this->m_customDispatch = std::make_shared<robotics::gazebotransport::CustomMsgDispatcher>();

        this->m_customDispatch->registerCustomHandler(
            std::make_shared<robotics::gazebotransport::TestPoseMsgHandler>(this->nodeSub),
            "gazebo_msgs/TestPose");

        // false message type registered to check gazebo topic type validation
        this->m_customDispatch->registerCustomHandler(
            std::make_shared<robotics::gazebotransport::TestPoseMsgHandler>(this->nodeSub),
            "gazebo_msgs/TestPoseFalse");

        this->m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::SubscribeCustomMsgHandler>(
                this->m_customDispatch));

        this->m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::InitSubscribeCustomMsgHandler>(
                this->m_customDispatch));

        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    void TearDown() {
        /// Close Threads
        this->clientEnd = 0;
        th0->join();
        th1->join();
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // Create Packet message with INIT_CUSTOM_MESSAGE_SUBSCRIBER
    mw::internal::robotics::gazebotransport::Packet createInitSubsciberMessage(
        std::string const& topic_name,
        std::string const& message_type) {
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_SUBSCRIBER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_init_custom_subscriber()->set_topic_name(topic_name);
        m_message.mutable_init_custom_subscriber()->set_message_type(message_type);

        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    // Create Packet message with CUSTOM_MESSAGE_SUBSCRIBER
    mw::internal::robotics::gazebotransport::Packet createSubscribeRequestMessage(
        std::string const& topic_name,
        std::string const& message_type) {
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_custom_message_support()->set_topic_name(topic_name);
        m_message.mutable_request_custom_message_support()->set_message_type(message_type);

        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }
};

/*
 * Subscribe and Get custom message data 0
 * It tests the subscriber is successfully initialized or not.
 * Also, it tests the client successfully gets the custom message 0 or not.
 * The verifications are done with ground truth values.
 */
TEST_F(customMessageSubscribe, testCustomMessageSubscribeSuccess) {
    //*********************************************
    /// TEST Init Custom Subscriber

    mw::internal::robotics::gazebotransport::Packet reply =
        createInitSubsciberMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    bool m_success = false;

    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    //*********************************************
    /// TEST Subscribe Custom message

    mw::internal::robotics::gazebotransport::Packet reply0 =
        createSubscribeRequestMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    TestPose m_reply_pose;
    bool m_success0 = false;

    if (reply0.has_custom_message_support()) {
        m_reply_pose.ParseFromString(reply0.custom_message_support().data());
        m_success0 = true;
    }

    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success0);

    TestPose ground_pose0;
    ground_pose0.set_x(10);
    ground_pose0.set_y(20);
    ground_pose0.set_z(30);
    ground_pose0.set_w(40);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_reply_pose, ground_pose0));
}

// Initialize two custom subscriber for two topic of same message type.
// Further, subscribe message on corresponding topic and validate message
TEST_F(customMessageSubscribe, testTwoCustomMessageSubscribeSuccess) {
    //*********************************************
    /// TEST Init Custom Subscriber 0

    mw::internal::robotics::gazebotransport::Packet m_reply0 =
        createInitSubsciberMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    bool m_success = false;

    if (m_reply0.has_status() && !m_reply0.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    //*********************************************
    /// TEST Init Custom Subscriber 1

    mw::internal::robotics::gazebotransport::Packet m_reply1 =
        createInitSubsciberMessage("/gazebo/default/test_topic1", "gazebo_msgs/TestPose");


    if (m_reply1.has_status() && !m_reply1.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    //*********************************************
    /// TEST Subscribe Custom message 0

    mw::internal::robotics::gazebotransport::Packet reply0 =
        createSubscribeRequestMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    TestPose m_reply_pose;
    bool m_success0 = false;

    if (reply0.has_custom_message_support()) {
        m_reply_pose.ParseFromString(reply0.custom_message_support().data());
        m_success0 = true;
    }

    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success0);

    TestPose ground_pose0;
    ground_pose0.set_x(10);
    ground_pose0.set_y(20);
    ground_pose0.set_z(30);
    ground_pose0.set_w(40);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_reply_pose, ground_pose0));

    std::this_thread::sleep_for(std::chrono::microseconds(5000));
    //*********************************************
    /// TEST Subscribe Custom message 1

    mw::internal::robotics::gazebotransport::Packet reply1 =
        createSubscribeRequestMessage("/gazebo/default/test_topic1", "gazebo_msgs/TestPose");

    TestPose m_reply_pose1;
    bool m_success1 = false;

    if (reply1.has_custom_message_support()) {
        m_reply_pose1.ParseFromString(reply1.custom_message_support().data());
        m_success1 = true;
    }

    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success1);

    TestPose ground_pose1;
    ground_pose1.set_x(40);
    ground_pose1.set_y(30);
    ground_pose1.set_z(20);
    ground_pose1.set_w(10);

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_reply_pose1, ground_pose1));
}

/*
 * Initialize Custom Message Subscriber returns error message for invalid custom message type.
 * The custom message type should be same as registered custom message type.
 */
TEST_F(customMessageSubscribe, testInitSubscribeInvalidMessageType) {
    //*********************************************
    /// TEST Init Custom Subscriber

    mw::internal::robotics::gazebotransport::Packet reply = createInitSubsciberMessage(
        "/gazebo/default/test_topic0", "gazebo_msgs/Invalid_Message_Type");

    bool m_success = false;

    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be false as message type is not registered
    ASSERT_FALSE(m_success);
}

/*
 * Subscribe Custom Message returns error message for invalid custom message type.
 * The custom message type should be same as registered custom message type.
 */
TEST_F(customMessageSubscribe, testSubscribeInvalidMessageType) {
    //*********************************************
    /// TEST Init Custom Subscriber

    mw::internal::robotics::gazebotransport::Packet reply =
        createInitSubsciberMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    bool m_success = false;

    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    //*********************************************
    /// TEST Subscribe Custom message

    mw::internal::robotics::gazebotransport::Packet reply0 = createSubscribeRequestMessage(
        "/gazebo/default/test_topic0", "gazebo_msgs/Invalid_Message_Type");

    TestPose m_reply_pose;
    bool m_success0 = false;

    if (reply0.has_custom_message_support() &&
        m_reply_pose.ParseFromString(reply0.custom_message_support().data())) {
        m_success0 = true;
    }

    // it should be false as message type is not registered
    ASSERT_FALSE(m_success0);

    TestPose ground_pose0;
    ground_pose0.set_x(10);
    ground_pose0.set_y(20);
    ground_pose0.set_z(30);
    ground_pose0.set_w(40);

    EXPECT_FALSE(google::protobuf::util::MessageDifferencer::Equals(m_reply_pose, ground_pose0));
}

/*
 * Subscribe Custom Message returns error message for invalid custom topic name.
 * The custom topic should be initialized by init subscriber.
 */
TEST_F(customMessageSubscribe, testSubscribeInvalidTopicName) {
    //*********************************************
    /// TEST Init Custom Subscriber

    mw::internal::robotics::gazebotransport::Packet reply =
        createInitSubsciberMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    bool m_success = false;

    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    //*********************************************
    /// TEST Subscribe Custom message

    mw::internal::robotics::gazebotransport::Packet reply0 =
        createSubscribeRequestMessage("/gazebo/default/invalid_topic", "gazebo_msgs/TestPose");

    TestPose m_reply_pose;
    bool m_success0 = false;

    if (reply0.has_custom_message_support() &&
        m_reply_pose.ParseFromString(reply0.custom_message_support().data())) {
        m_success0 = true;
    }

    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(m_success0);

    TestPose ground_pose0;
    ground_pose0.set_x(10);
    ground_pose0.set_y(20);
    ground_pose0.set_z(30);
    ground_pose0.set_w(40);

    EXPECT_FALSE(google::protobuf::util::MessageDifferencer::Equals(m_reply_pose, ground_pose0));
}


/*
 * Subscribe Custom Message returns error message for
 * initialize same topic name again
 */
TEST_F(customMessageSubscribe, testInitSubscriberSameTopicAgain) {
    //*********************************************
    /// TEST Init Custom Subscriber

    mw::internal::robotics::gazebotransport::Packet reply =
        createInitSubsciberMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    bool m_success = false;

    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    //*********************************************
    /// TEST Init Custom Subscriber again

    reply = createInitSubsciberMessage("/gazebo/default/test_topic0", "gazebo_msgs/TestPose");

    m_success = false;

    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be false as topic is already initialized
    ASSERT_TRUE(m_success);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
