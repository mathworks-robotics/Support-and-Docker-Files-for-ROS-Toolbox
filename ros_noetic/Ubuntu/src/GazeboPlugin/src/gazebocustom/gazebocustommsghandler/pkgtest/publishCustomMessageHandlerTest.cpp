/* Copyright 2019-2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"

#include "gazebotransport/gazebocustom/gazebocustommsghandler/PublishCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/InitPublishCustomMsgHandler.hpp"
#include "TestPoseMsgHandler.hpp"
#include "mw.internal.robotics.gazebotransport.TestMsgs.pb.h"

#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "google/protobuf/util/message_differencer.h"
#include <gazebo/physics/physics.hh>

using namespace mw::internal::robotics::gazebotransport;

typedef const boost::shared_ptr<const TestPose> TestPosePtr;

class customMessagePublish : public testing::Test {

  public:
    std::string ipAddress = "127.0.0.1";

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /// Session Time-out value
    int time_out = 5000;

    // Gazebo Node pointer
    gazebo::transport::NodePtr nodeSub;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr world;

    // Custom message dispatcher instance
    std::shared_ptr<robotics::gazebotransport::CustomMsgDispatcher> m_customDispatch;

    // Gazebo subscriber pointer to validate published message
    gazebo::transport::SubscriberPtr subPtr0;
    gazebo::transport::SubscriberPtr subPtr1;

    // Received test pose message
    TestPose m_receivedPose0;
    TestPose m_receivedPose1;

    std::mutex m_mutex;

    // Callback to Subscribe custom message
    void subscribeMessageCallback0(TestPosePtr& msg) {
        // copy received message
        std::lock_guard<std::mutex> lock(m_mutex);
        m_receivedPose0.CopyFrom(*msg);
    }

    // Callback to Subscribe custom message
    void subscribeMessageCallback1(TestPosePtr& msg) {
        // copy received message
        std::lock_guard<std::mutex> lock(m_mutex);
        m_receivedPose1.CopyFrom(*msg);
    }

    // Start custom message Subscriber0
    void startSubscriber0() {
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        subPtr0 = node->Subscribe("~/test_topic0", &customMessagePublish::subscribeMessageCallback0,
                                  this);
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }

    // Start custom message Subscriber1
    void startSubscriber1() {
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        subPtr1 = node->Subscribe("~/test_topic1", &customMessagePublish::subscribeMessageCallback1,
                                  this);
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
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
        world = gazebo::loadWorld("world/unitBoxHandlerTest.world");
        ASSERT_TRUE(world != nullptr) << "Failed to load world";
        world->SetPaused(true);

        /// Init Node, SubPtr and GazeboMsgRepo for msgHandler
        nodeSub = gazebo::transport::NodePtr(new gazebo::transport::Node());
        nodeSub->Init();

        // start subscribers
        startSubscriber0();
        startSubscriber1();

        /// Init custom message dispatcher
        this->m_customDispatch = std::make_shared<robotics::gazebotransport::CustomMsgDispatcher>();

        // Starts custom message Test handler
        this->m_customDispatch->registerCustomHandler(
            std::make_shared<robotics::gazebotransport::TestPoseMsgHandler>(this->nodeSub),
            "gazebo_msgs/TestPose");

        // Starts InitPublish custom message handler
        this->m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::PublishCustomMsgHandler>(
                this->m_customDispatch));

        // Starts Publish custom message handler
        this->m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::InitPublishCustomMsgHandler>(
                this->m_customDispatch));

        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    void TearDown() {
        // clear received messages
        m_receivedPose0.Clear();
        m_receivedPose1.Clear();

        std::this_thread::sleep_for(std::chrono::microseconds(5000));
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();

        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // Create Packet message with INIT_CUSTOM_MESSAGE_PUBLISHER
    mw::internal::robotics::gazebotransport::Packet createInitPublisherMessage(
        std::string const& topic_name,
        std::string const& message_type) {
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_PUBLISHER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_init_custom_publisher()->set_topic_name(topic_name);
        m_message.mutable_init_custom_publisher()->set_message_type(message_type);

        // Serialize message, send to server and wait for reply
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        // deserialize Packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }
        return reply;
    }


    // Create Packet message with CUSTOM_MESSAGE_PUBLISHER
    mw::internal::robotics::gazebotransport::Packet createPublishMessage(
        std::string const& topic_name,
        std::string const& message_type,
        std::string const& data) {

        mw::internal::robotics::gazebotransport::Packet n_message;
        n_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_CUSTOM_MESSAGE_PUBLISHER);
        n_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        n_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        n_message.mutable_custom_message_support()->set_topic_name(topic_name);
        n_message.mutable_custom_message_support()->set_message_type(message_type);

        // Fill packet message data field with serialized custom message
        n_message.mutable_custom_message_support()->set_data(data);

        // Serialize message, send to server and wait for reply
        auto msg0 = n_message.SerializeAsString();
        auto n_replyMsg = m_client->write(msg0, boost::posix_time::milliseconds(time_out));

        // deserialize into Packet message
        mw::internal::robotics::gazebotransport::Packet n_reply;
        if (n_replyMsg) {
            n_reply.ParseFromString(*n_replyMsg);
        }

        return n_reply;
    }
};


/*
 * Initialize Custom Message Publisher and Publish custom message on specified topic.
 * It tests the Publisher is successfully initialized or not.
 * Also, it tests the message successfully published or not.
 * The verifications are done with fixed message values.
 */
TEST_F(customMessagePublish, testCustomMessagePublishSuccess) {

    //*********************************************
    // deserialize Packet message
    mw::internal::robotics::gazebotransport::Packet reply =
        createInitPublisherMessage("~/test_topic0", "gazebo_msgs/TestPose");

    // Success of reply message
    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    std::this_thread::sleep_for(std::chrono::microseconds(50000));

    //*********************************************
    /// TEST Publish Custom Message

    // This message used to publish on specific topic
    TestPose m_pose;
    // Initialize pose message ( Custom Message )
    m_pose.set_x(50);
    m_pose.set_y(20);
    m_pose.set_z(80);
    m_pose.set_w(60);

    // deserialize into Packet message
    mw::internal::robotics::gazebotransport::Packet n_reply =
        createPublishMessage("~/test_topic0", "gazebo_msgs/TestPose", m_pose.SerializeAsString());

    // Success of reply message
    bool n_success = false;
    if (n_reply.has_status() && !n_reply.status()) {
        n_success = true;
    }

    // it should be true as topic is available in gazebo
    ASSERT_TRUE(n_success);

    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_pose, m_receivedPose0));
}


// Initialize two custom publisher for two topic of same message type.
// Further, publish message on corresponding topic and validate message
TEST_F(customMessagePublish, testTwoCustomMessagePublishSuccess) {

    //*********************************************
    // Initialize custom publisher 0
    mw::internal::robotics::gazebotransport::Packet reply =
        createInitPublisherMessage("~/test_topic0", "gazebo_msgs/TestPose");

    // Success of reply message
    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    std::this_thread::sleep_for(std::chrono::microseconds(50000));

    //*********************************************
    // Initialize custom publisher 1
    reply = createInitPublisherMessage("~/test_topic1", "gazebo_msgs/TestPose");

    // Success of reply message
    m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    std::this_thread::sleep_for(std::chrono::microseconds(50000));

    //*********************************************
    /// TEST Publish Custom Message 0

    // This message used to publish on specific topic
    TestPose m_pose;
    // Initialize pose message ( Custom Message )
    m_pose.set_x(50);
    m_pose.set_y(20);
    m_pose.set_z(80);
    m_pose.set_w(60);

    // deserialize into Packet message
    mw::internal::robotics::gazebotransport::Packet n_reply =
        createPublishMessage("~/test_topic0", "gazebo_msgs/TestPose", m_pose.SerializeAsString());

    // Success of reply message
    bool n_success = false;
    if (n_reply.has_status() && !n_reply.status()) {
        n_success = true;
    }

    // it should be true as topic is available in gazebo
    ASSERT_TRUE(n_success);

    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_pose, m_receivedPose0));

    //*********************************************
    /// TEST Publish Custom Message 1

    // This message used to publish on specific topic
    TestPose m_pose1;
    // Initialize pose message ( Custom Message )
    m_pose1.set_x(10);
    m_pose1.set_y(30);
    m_pose1.set_z(40);
    m_pose1.set_w(20);

    // deserialize into Packet message
    n_reply =
        createPublishMessage("~/test_topic1", "gazebo_msgs/TestPose", m_pose1.SerializeAsString());

    // Success of reply message
    n_success = false;
    if (n_reply.has_status() && !n_reply.status()) {
        n_success = true;
    }

    // it should be true as topic is available in gazebo
    ASSERT_TRUE(n_success);

    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_pose1, m_receivedPose1));
}


/*
 * Initialize Custom Message Publisher returns error message for invalid custom message type.
 * The custom message type should be same as registered custom message type.
 */
TEST_F(customMessagePublish, testInitPublishInvalidMessageType) {
    //*********************************************
    // Initialize custom publisher
    mw::internal::robotics::gazebotransport::Packet reply =
        createInitPublisherMessage("~/test_topic0", "gazebo_msgs/Invalid_Type_Pose");

    // Success of reply message
    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be false as message type is not registered
    ASSERT_FALSE(m_success);
}

/*
 * Publish Custom Message returns error message for invalid custom message type.
 * The custom message type should be same as registered custom message type.
 */
TEST_F(customMessagePublish, testPublishInvalidMessageType) {
    //*********************************************
    // Initialize custom publisher
    mw::internal::robotics::gazebotransport::Packet reply =
        createInitPublisherMessage("~/test_topic0", "gazebo_msgs/TestPose");

    // Success of reply message
    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    //*********************************************
    /// TEST Publish Custom Message

    // This message used to publish on specific topic
    TestPose m_pose;
    // Initialize pose message ( Custom Message )
    m_pose.set_x(50);
    m_pose.set_y(20);
    m_pose.set_z(80);
    m_pose.set_w(60);

    mw::internal::robotics::gazebotransport::Packet n_reply = createPublishMessage(
        "~/test_topic0", "gazebo_msgs/Invalid_Type_Pose", m_pose.SerializeAsString());

    // Success of reply message
    bool n_success = false;
    if (n_reply.has_status() && !n_reply.status()) {
        n_success = true;
    }

    // it should be false as message type is not registered
    ASSERT_FALSE(n_success);

    ASSERT_FALSE(google::protobuf::util::MessageDifferencer::Equals(m_pose, m_receivedPose0));
}

/*
 * Publish Custom Message returns error message for invalid custom topic name.
 * The custom topic should be initialized by init publisher.
 */
TEST_F(customMessagePublish, testPublishInvalidTopicName) {
    //*********************************************
    // Initialize custom publisher
    mw::internal::robotics::gazebotransport::Packet reply =
        createInitPublisherMessage("~/test_topic0", "gazebo_msgs/TestPose");

    // Success of reply message
    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    //*********************************************
    /// TEST Publish Custom Message

    // This message used to publish on specific topic
    TestPose m_pose;
    // Initialize pose message ( Custom Message )
    m_pose.set_x(50);
    m_pose.set_y(20);
    m_pose.set_z(80);
    m_pose.set_w(60);

    mw::internal::robotics::gazebotransport::Packet n_reply =
        createPublishMessage("~/invalid_topic", "gazebo_msgs/TestPose", m_pose.SerializeAsString());

    // Success of reply message
    bool n_success = false;
    if (n_reply.has_status() && !n_reply.status()) {
        n_success = true;
    }

    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(n_success);

    ASSERT_FALSE(google::protobuf::util::MessageDifferencer::Equals(m_pose, m_receivedPose0));
}

/*
 * Publish Custom Message returns error message for invalid custom message.
 * The custom topic should be initialized by init publisher.
 */
TEST_F(customMessagePublish, testPublishInvalidCustomMessage) {
    //*********************************************
    // Initialize custom publisher
    mw::internal::robotics::gazebotransport::Packet reply =
        createInitPublisherMessage("~/test_topic0", "gazebo_msgs/TestPose");

    // Success of reply message
    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    //*********************************************
    /// TEST Publish Custom Message

    // The message is not valid custom message
    mw::internal::robotics::gazebotransport::Packet invalid_message;
    invalid_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    invalid_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    invalid_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    invalid_message.set_status(
        mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    mw::internal::robotics::gazebotransport::Packet n_reply = createPublishMessage(
        "~/test_topic0", "gazebo_msgs/TestPose", invalid_message.SerializeAsString());

    // Success of reply message
    bool n_success = false;
    if (n_reply.has_status() && !n_reply.status()) {
        n_success = true;
    }

    // it should be false as published custom message is invalid
    ASSERT_FALSE(n_success);
}

/*
 * Publish Custom Message returns error message for already initialized custom message topic.
 */
TEST_F(customMessagePublish, testInitPublisherSameTopicAgain) {
    //*********************************************
    // Initialize custom publisher
    mw::internal::robotics::gazebotransport::Packet reply =
        createInitPublisherMessage("~/test_topic0", "gazebo_msgs/TestPose");

    // Success of reply message
    bool m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);

    //*********************************************
    // Initialize custom publisher again
    reply = createInitPublisherMessage("~/test_topic0", "gazebo_msgs/TestPose");

    // Success of reply message
    m_success = false;
    if (reply.has_status() && !reply.status()) {
        m_success = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
