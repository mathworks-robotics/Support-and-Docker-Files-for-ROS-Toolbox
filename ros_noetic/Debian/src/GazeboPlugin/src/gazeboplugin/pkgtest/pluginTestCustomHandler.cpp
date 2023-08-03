/* Copyright 2019-2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/Client.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GazeboUtils.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include "mw.internal.robotics.gazebotransport.TestMsgs.pb.h"
#include "image_mat.pb.h"
#include "imu_mat.pb.h"
#include "google/protobuf/util/message_differencer.h"

#include <mutex>

using namespace mw::internal::robotics::gazebotransport;

/*
This code tests the custom message handler functionalities.
The built-in as well as user-defined custom message are
tested in this module.

*/
typedef const boost::shared_ptr<const robotics::matlab::ImageMat> ImageMatPtr;
typedef const boost::shared_ptr<const robotics::matlab::IMUMat> IMUMatPtr;

class GazeboWorldStat {
  public:
    GazeboWorldStat()
        : m_mutex()
        , m_time()
        , m_worldStat(nullptr) {
    }

    void OnWorldStat(ConstWorldStatisticsPtr& msg) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_time = gazebo::msgs::Convert(msg->sim_time());
    }

    gazebo::common::Time getSimTime() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_time;
    }

    void setSubscriber(gazebo::transport::SubscriberPtr&& sub) {
        m_worldStat = sub;
    }

  private:
    /// mutex
    std::mutex m_mutex;
    /// Simulation time holder
    gazebo::common::Time m_time;
    /// Gazebo Subscriber pointer to Subscribe world stat data
    gazebo::transport::SubscriberPtr m_worldStat;
};

class pluginTestCustomHandler : public testing::Test {
  public:
    std::string ipAddress = "127.0.0.1";
    std::string serverPort = "14581";

    /// Session Time-out value
    int time_out = 50000;

    /// Creating Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;

    /// Ignition transport node
    gazebo::transport::NodePtr m_node;

    // Gazebo subscriber pointer to validate published message
    gazebo::transport::SubscriberPtr subPtr0;
    gazebo::transport::SubscriberPtr subPtr1;

    /// Stores world stat and simulation time from Gazebo
    GazeboWorldStat m_worldStat;

    /// Threads to publish custom message
    std::shared_ptr<std::thread> th0;
    std::shared_ptr<std::thread> th1;

    std::mutex m_mutex;

    // Received test pose message
    robotics::matlab::ImageMat m_receivedPose0;
    robotics::matlab::IMUMat m_receivedPose1;

    void SetUp() {
        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, serverPort, boost::posix_time::milliseconds(time_out));

        /// Start Gazebo Simulator client
        gazebo::client::setup();

        // setup ignition transport node and subscriber
        m_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        m_node->Init();
        m_worldStat.setSubscriber(
            m_node->Subscribe("~/world_stats", &GazeboWorldStat::OnWorldStat, &m_worldStat));

        mw::internal::robotics::gazebotransport::Packet reply = clientRequestCoSimulation("TestID");
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as the server would grant request
        ASSERT_TRUE(success);

        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }

    void TearDown() {
        mw::internal::robotics::gazebotransport::Packet reply = clientStopCoSimulation("TestID");
        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        } else {
            success = false;
        }
        /// it should be true as the server would stop co-simulation
        ASSERT_TRUE(success);

        m_client->shutdown();

        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }

    /**
     * It creates and sends request co-simulation message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientRequestCoSimulation(
        std::string const& clientID) {
        /// Create Packet message request co-simulation message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_REQUEST_COSIM);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_cosim()->set_client_id(clientID);
        m_message.mutable_request_cosim()->set_duration(std::numeric_limits<double>::infinity());
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends stop co-simulation message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientStopCoSimulation(
        std::string const& clientID) {
        /// Create Packet message stop co-simulation message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STOP_COSIM);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_stop_cosim()->set_client_id(clientID);
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends reset gazebo scene & time message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientResetSceneTime() {

        /// Create Packet message reset simulation time & scene message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_RESET_SIMULATION);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_reset_simulation()->set_behavior(
            mw::internal::robotics::gazebotransport::
                ResetSimulation_ResetBehavior_RESET_TIME_AND_SCENE);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends step simulation message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientStepSimulation(uint32_t stepSize) {
        /// Create Packet message step simulation message
        mw::internal::robotics::gazebotransport::Packet m_message;

        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STEP_SIMULATION);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_step_simulation()->set_num_steps(stepSize);
        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;

        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends init subscriber message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
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

    /**
     * It creates and sends subscribe custom message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
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

    /**
     * It creates and sends init publisher message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
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

    /**
     * It creates and sends publish custom message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
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

    /// Reset Scene Test module
    void resetScene() {
        /// Reset gazebo scene and time
        mw::internal::robotics::gazebotransport::Packet reply = clientResetSceneTime();

        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as reset done
        ASSERT_TRUE(success);
    }

    /// Step Scene Test module
    void stepScene(uint32_t stepSize) {
        /// step gazebo by one
        mw::internal::robotics::gazebotransport::Packet reply = clientStepSimulation(stepSize);

        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }
        /// it should be true as step is done
        ASSERT_TRUE(success);
    }

    /// Init Subscriber Test module
    void initSubscriberTest(std::string const& topic_name,
                            std::string const& message_type,
                            bool expected) {
        /// init custom message subscriber
        mw::internal::robotics::gazebotransport::Packet reply =
            createInitSubsciberMessage(topic_name, message_type);

        bool success = false;
        if (reply.has_status() && !reply.status()) {
            success = true;
        }

        /// Check expected result
        if (expected) {
            ASSERT_TRUE(success);
        } else {
            ASSERT_FALSE(success);
        }
    }

    /// Init Publisher Test module
    void initPublisherTest(std::string const& topic_name,
                           std::string const& message_type,
                           bool expected) {
        /// init custom message publisher
        mw::internal::robotics::gazebotransport::Packet reply =
            createInitPublisherMessage(topic_name, message_type);

        bool success = false;

        if (reply.has_status() && !reply.status()) {
            success = true;
        }

        /// Check expected result
        if (expected) {
            ASSERT_TRUE(success);
        } else {
            ASSERT_FALSE(success);
        }
    }

    /// Subscriber Custom Message Test module
    void subscribeMessageTest(std::string const& topic_name,
                              std::string const& message_type,
                              TestPose ground_msg,
                              bool expected) {
        // subscribe custom message
        mw::internal::robotics::gazebotransport::Packet replyMsg =
            createSubscribeRequestMessage(topic_name, message_type);

        TestPose m_reply;
        bool success = false;
        if (replyMsg.has_custom_message_support()) {
            if (m_reply.ParseFromString(replyMsg.custom_message_support().data())) {
                success = true;
            }
        }

        // Check expected result
        if (expected) {
            ASSERT_TRUE(success);
            EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_reply, ground_msg));
        } else {
            ASSERT_FALSE(success);
            EXPECT_FALSE(google::protobuf::util::MessageDifferencer::Equals(m_reply, ground_msg));
        }
    }

    // Publish Custom Message Test module
    void publishMessageTest(std::string const& topic_name,
                            std::string const& message_type,
                            TestPose ground_msg,
                            bool expected) {
        // publish custom message
        mw::internal::robotics::gazebotransport::Packet reply0 =
            createPublishMessage(topic_name, message_type, ground_msg.SerializeAsString());

        bool success = false;

        if (reply0.has_status() && !reply0.status()) {
            success = true;
        }

        // Check expected result
        if (expected) {
            ASSERT_TRUE(success);
        } else {
            ASSERT_FALSE(success);
        }
    }

    // Publish Image Message
    void publishMessageImage(std::string const& topic_name,
                             std::string const& message_type,
                             robotics::matlab::ImageMat ground_msg,
                             bool expected) {
        // publish custom message
        mw::internal::robotics::gazebotransport::Packet reply0 =
            createPublishMessage(topic_name, message_type, ground_msg.SerializeAsString());

        bool success = false;

        if (reply0.has_status() && !reply0.status()) {
            success = true;
        }

        // Check expected result
        if (expected) {
            ASSERT_TRUE(success);
        } else {
            ASSERT_FALSE(success);
        }
    }

    // Publish IMU Message
    void publishMessageIMU(std::string const& topic_name,
                           std::string const& message_type,
                           robotics::matlab::IMUMat ground_msg,
                           bool expected) {
        // publish custom message
        mw::internal::robotics::gazebotransport::Packet reply0 =
            createPublishMessage(topic_name, message_type, ground_msg.SerializeAsString());

        bool success = false;

        if (reply0.has_status() && !reply0.status()) {
            success = true;
        }

        // Check expected result
        if (expected) {
            ASSERT_TRUE(success);
        } else {
            ASSERT_FALSE(success);
        }
    }

    // Subscriber Built-In ImageStamped Message Test module
    void subscribeBuiltInImageMessageTest(std::string const& topic_name,
                                          std::string const& message_type,
                                          robotics::matlab::ImageMat ground_msg,
                                          bool expected) {
        // subscribe custom message
        mw::internal::robotics::gazebotransport::Packet replyMsg =
            createSubscribeRequestMessage(topic_name, message_type);

        robotics::matlab::ImageMat m_reply;
        bool success = false;
        if (replyMsg.has_custom_message_support()) {
            if (m_reply.ParseFromString(replyMsg.custom_message_support().data())) {
                success = true;
            }
        }

        // Check expected result
        if (expected) {
            ASSERT_TRUE(success);
            EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_reply, ground_msg));
        } else {
            ASSERT_FALSE(success);
        }
    }

    // Subscriber Built-In IMU Message Test module
    void subscribeBuiltInIMUMessageTest(std::string const& topic_name,
                                        std::string const& message_type,
                                        robotics::matlab::IMUMat ground_msg,
                                        bool expected) {
        // subscribe custom message
        mw::internal::robotics::gazebotransport::Packet replyMsg =
            createSubscribeRequestMessage(topic_name, message_type);

        robotics::matlab::IMUMat m_reply;
        bool success = false;
        if (replyMsg.has_custom_message_support()) {
            if (m_reply.ParseFromString(replyMsg.custom_message_support().data())) {
                success = true;
            }
        }

        // Check expected result
        if (expected) {
            ASSERT_TRUE(success);
            EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(m_reply, ground_msg));
        } else {
            ASSERT_FALSE(success);
        }
    }
};

/*
 * Test built-in image message subscribe
 */
TEST_F(pluginTestCustomHandler, testBuiltInImageSubscribeMessageSuccess) {

    resetScene();

    initSubscriberTest("/gazebo/default/custom_image_publish",
                       "gazebo_msgs/robotics_matlab/ImageMat", true);

    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {
        robotics::matlab::ImageMat ground_msg;
        ground_msg.set_m_width(320);
        ground_msg.set_m_height(240);
        ground_msg.set_m_pixel_format(3);
        ground_msg.set_m_step(960);
        ground_msg.set_m_data("\255\255\255\255\255");

        if (itr > 0) {
            subscribeBuiltInImageMessageTest("/gazebo/default/custom_image_publish",
                                             "gazebo_msgs/robotics_matlab/ImageMat", ground_msg,
                                             true);
        }

        stepScene(1);
    }
}

/**
 * Publish Built-In Image Message
 */
TEST_F(pluginTestCustomHandler, testBuiltInImagePublishMessageSuccess) {
    resetScene();
    initPublisherTest("~/custom_image_subscribe", "gazebo_msgs/robotics_matlab/ImageMat", true);

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {
        stepScene(1);

        robotics::matlab::ImageMat ground_msg;
        ground_msg.set_m_width(320);
        ground_msg.set_m_height(240);
        ground_msg.set_m_pixel_format(3);
        ground_msg.set_m_step(960);
        ground_msg.set_m_data("\255\255\255\255\255");

        publishMessageImage("~/custom_image_subscribe", "gazebo_msgs/robotics_matlab/ImageMat",
                            ground_msg, true);
    }
}

/*
 * Test built-in IMU message subscribe
 */
TEST_F(pluginTestCustomHandler, testBuiltInIMUSubscribeMessageSuccess) {

    resetScene();

    initSubscriberTest("/gazebo/default/custom_imu_publish", "gazebo_msgs/robotics_matlab/IMUMat",
                       true);

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {
        robotics::matlab::IMUMat ground_msg;
        ground_msg.set_m_entity_name("IMU");
        ground_msg.mutable_m_angular_velocity()->set_m_x(0.05);
        ground_msg.mutable_m_angular_velocity()->set_m_y(0.01);
        ground_msg.mutable_m_angular_velocity()->set_m_z(0.01);
        ground_msg.mutable_m_linear_acceleration()->set_m_x(1.00);
        ground_msg.mutable_m_linear_acceleration()->set_m_y(2.00);
        ground_msg.mutable_m_linear_acceleration()->set_m_z(3.00);
        ground_msg.mutable_m_orientation()->set_m_x(0.1);
        ground_msg.mutable_m_orientation()->set_m_y(0.2);
        ground_msg.mutable_m_orientation()->set_m_z(0.3);
        ground_msg.mutable_m_orientation()->set_m_w(0.4);
        ground_msg.mutable_m_stamp()->set_m_nsec(1000);
        ground_msg.mutable_m_stamp()->set_m_sec(1);

        if (itr > 0) {
            subscribeBuiltInIMUMessageTest("/gazebo/default/custom_imu_publish",
                                           "gazebo_msgs/robotics_matlab/IMUMat", ground_msg, true);
        }

        stepScene(1);
    }
}

/**
 * Publish Built-In IMU Message
 */
TEST_F(pluginTestCustomHandler, testBuiltInIMUPublishMessageSuccess) {
    resetScene();
    initPublisherTest("~/custom_imu_subscribe", "gazebo_msgs/robotics_matlab/IMUMat", true);

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {
        stepScene(1);

        robotics::matlab::IMUMat ground_msg;
        ground_msg.set_m_entity_name("IMU");
        ground_msg.mutable_m_angular_velocity()->set_m_x(0.05);
        ground_msg.mutable_m_angular_velocity()->set_m_y(0.01);
        ground_msg.mutable_m_angular_velocity()->set_m_z(0.01);
        ground_msg.mutable_m_linear_acceleration()->set_m_x(1.00);
        ground_msg.mutable_m_linear_acceleration()->set_m_y(2.00);
        ground_msg.mutable_m_linear_acceleration()->set_m_z(3.00);
        ground_msg.mutable_m_orientation()->set_m_x(0.1);
        ground_msg.mutable_m_orientation()->set_m_y(0.2);
        ground_msg.mutable_m_orientation()->set_m_z(0.3);
        ground_msg.mutable_m_orientation()->set_m_w(0.4);
        ground_msg.mutable_m_stamp()->set_m_nsec(1000);
        ground_msg.mutable_m_stamp()->set_m_sec(1);

        publishMessageIMU("~/custom_imu_subscribe", "gazebo_msgs/robotics_matlab/IMUMat",
                          ground_msg, true);
    }
}

/*
 * Initialize and Subscribe custom message data 0
 * It tests the subscriber is successfully initialized or not.
 * Also, it tests the client successfully gets the custom message 0 or not.
 * The verifications are done with ground truth values.
 */
TEST_F(pluginTestCustomHandler, testCustomMessageSubscribeSuccess) {

    resetScene();

    initSubscriberTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose", true);

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {
        // Message published from plugin on step 1
        if (itr > 0) {
            TestPose ground_msg;
            double itr_num = 100.0 + (double)itr - 1.0;
            ground_msg.set_x(itr_num);
            ground_msg.set_y(itr_num);
            ground_msg.set_z(itr_num);
            ground_msg.set_w(itr_num);

            subscribeMessageTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose",
                                 ground_msg, true);
        }

        stepScene(1);
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
}

/*
 * Initialize and Subscribe custom message data 0 and data 1
 * It tests two subscribers are successfully initialized or not.
 * Also, it tests the client successfully gets the custom message 0
 * and custom message 1 or not.
 * The verifications are done with ground truth values.
 */
TEST_F(pluginTestCustomHandler, testTwoCustomMessageSubscribeSuccess) {
    resetScene();

    // false as already initialized in testCustomMessageSubscribeSuccess TEST
    initSubscriberTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose", true);

    initSubscriberTest("/gazebo/default/test_pose_publisher1", "gazebo_msgs/TestPose", true);

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {
        // Message published from plugin on step 1
        if (itr > 0) {
            TestPose ground_msg;
            double itr_num0 = 100.0 + (double)itr - 1.0;
            ground_msg.set_x(itr_num0);
            ground_msg.set_y(itr_num0);
            ground_msg.set_z(itr_num0);
            ground_msg.set_w(itr_num0);

            subscribeMessageTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose",
                                 ground_msg, true);

            TestPose ground_pose1;
            double itr_num1 = 200.0 + (double)itr - 1.0;
            ground_pose1.set_x(itr_num1);
            ground_pose1.set_y(itr_num1);
            ground_pose1.set_z(itr_num1);
            ground_pose1.set_w(itr_num1);

            subscribeMessageTest("/gazebo/default/test_pose_publisher1", "gazebo_msgs/TestPose",
                                 ground_pose1, true);
        }

        stepScene(1);
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
}

/**
 * First topic is valid and second topic is invalid
 * for init custom message subscriber
 */
TEST_F(pluginTestCustomHandler, testFirstValidTopicInitSubscribe) {

    resetScene();

    // false as already initialized in testCustomMessageSubscribeSuccess TEST
    initSubscriberTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose", true);

    initSubscriberTest("/gazebo/default/test_pose_publisher1_invalid", "gazebo_msgs/TestPose",
                       false);

    stepScene(1);

    // Needed as take time to publish message from plugin side
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    TestPose ground_msg;
    double itr_num0 = 100.0;
    ground_msg.set_x(itr_num0);
    ground_msg.set_y(itr_num0);
    ground_msg.set_z(itr_num0);
    ground_msg.set_w(itr_num0);

    subscribeMessageTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose", ground_msg,
                         true);

    TestPose ground_pose1;
    double itr_num1 = 200.0;
    ground_pose1.set_x(itr_num1);
    ground_pose1.set_y(itr_num1);
    ground_pose1.set_z(itr_num1);
    ground_pose1.set_w(itr_num1);

    subscribeMessageTest("/gazebo/default/test_pose_publisher1_invalid", "gazebo_msgs/TestPose",
                         ground_pose1, false);
}

/**
 * First topic is invalid and second topic is valid
 * for init custom message subscriber
 */

TEST_F(pluginTestCustomHandler, testSecondValidTopicInitSubscribe) {

    resetScene();

    initSubscriberTest("/gazebo/default/test_pose_publisher0_invalid", "gazebo_msgs/TestPose",
                       false);

    // false as already initialized in testTwoCustomMessageSubscribeSuccess TEST
    initSubscriberTest("/gazebo/default/test_pose_publisher1", "gazebo_msgs/TestPose", true);

    stepScene(1);

    // Needed as take time to publish message from plugin side
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    TestPose ground_msg;
    double itr_num0 = 100.0;
    ground_msg.set_x(itr_num0);
    ground_msg.set_y(itr_num0);
    ground_msg.set_z(itr_num0);
    ground_msg.set_w(itr_num0);

    subscribeMessageTest("/gazebo/default/test_pose_publisher0_invalid", "gazebo_msgs/TestPose",
                         ground_msg, false);

    TestPose ground_pose1;
    double itr_num1 = 200.0;
    ground_pose1.set_x(itr_num1);
    ground_pose1.set_y(itr_num1);
    ground_pose1.set_z(itr_num1);
    ground_pose1.set_w(itr_num1);

    subscribeMessageTest("/gazebo/default/test_pose_publisher1", "gazebo_msgs/TestPose",
                         ground_pose1, true);
}

/**
 * Trying to subscribe message from topic which is
 * a non-initialized custom subscriber
 */
TEST_F(pluginTestCustomHandler, testInvalidTopicSubscribe) {
    resetScene();

    // false as already initialized in testCustomMessageSubscribeSuccess TEST
    initSubscriberTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose", true);

    stepScene(1);

    // Needed as take time to publish message from plugin side
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    TestPose ground_pose1;
    double itr_num1 = 200.0;
    ground_pose1.set_x(itr_num1);
    ground_pose1.set_y(itr_num1);
    ground_pose1.set_z(itr_num1);
    ground_pose1.set_w(itr_num1);

    subscribeMessageTest("/gazebo/default/test_pose_publisher0_invalid", "gazebo_msgs/TestPose",
                         ground_pose1, false);
}

/**
 * Topic name is valid but invalid message type for init subscriber
 * Further, Topic name and Message type is valid for subscribe but
 * it should through error as not initialized
 */
TEST_F(pluginTestCustomHandler, testInvalidMessageTypeInitSubscribe) {
    resetScene();
    initSubscriberTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose_Invalid",
                       false);

    stepScene(1);
}

/**
 * Topic name is valid but invalid message type for subscribe message
 */
TEST_F(pluginTestCustomHandler, testInvalidMessageTypeSubscribeMessage) {
    resetScene();

    // false as already initialized in testCustomMessageSubscribeSuccess TEST
    initSubscriberTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose", true);

    stepScene(1);

    // Needed as take time to publish message from plugin side
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    TestPose ground_pose1;
    double itr_num1 = 100.0;
    ground_pose1.set_x(itr_num1);
    ground_pose1.set_y(itr_num1);
    ground_pose1.set_z(itr_num1);
    ground_pose1.set_w(itr_num1);

    subscribeMessageTest("/gazebo/default/test_pose_publisher0", "gazebo_msgs/TestPose_Invalid",
                         ground_pose1, false);
}

/*
 * Initialize and Publish custom message data 0
 * It tests the publisher is successfully initialized or not.
 * The verifications are done with received error message.
 */
TEST_F(pluginTestCustomHandler, testCustomMessagePublishSuccess) {

    resetScene();

    initPublisherTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose", true);

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {

        TestPose ground_msg;
        double itr_num = 100.0 + (double)itr;
        ground_msg.set_x(itr_num);
        ground_msg.set_y(itr_num);
        ground_msg.set_z(itr_num);
        ground_msg.set_w(itr_num);

        publishMessageTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose",
                           ground_msg, true);

        stepScene(1);
    }
}

/*
 * Initialize and Publish custom message data 0 and data 1
 * It tests two publishers are successfully initialized or not.
 * The verifications are done with received error message.
 */
TEST_F(pluginTestCustomHandler, testTwoCustomMessagePublishSuccess) {
    resetScene();

    // false as already initialized in testCustomMessagePublishSuccess TEST
    initPublisherTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose", true);

    initPublisherTest("/gazebo/default/test_pose_subscriber1", "gazebo_msgs/TestPose", true);

    /// TEST Subscribe Custom message for 50 steps
    for (int itr = 0; itr < 50; itr++) {

        TestPose ground_msg;
        double itr_num = 100.0 + (double)itr;
        ground_msg.set_x(itr_num);
        ground_msg.set_y(itr_num);
        ground_msg.set_z(itr_num);
        ground_msg.set_w(itr_num);

        publishMessageTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose",
                           ground_msg, true);

        TestPose ground_pose1;
        itr_num = 200.0 + (double)itr;
        ground_pose1.set_x(itr_num);
        ground_pose1.set_y(itr_num);
        ground_pose1.set_z(itr_num);
        ground_pose1.set_w(itr_num);

        publishMessageTest("/gazebo/default/test_pose_subscriber1", "gazebo_msgs/TestPose",
                           ground_pose1, true);

        stepScene(1);
    }
}

/**
 * Publish on not initialized topic
 * i.e. Try to publish on invalid topic
 */
TEST_F(pluginTestCustomHandler, testInvalidTopicPublish) {
    resetScene();

    // false as already initialized in testCustomMessagePublishSuccess TEST
    initPublisherTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose", true);

    stepScene(1);

    // Needed as take time to publish message from plugin side
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    TestPose ground_msg;
    double itr_num = 200.0;
    ground_msg.set_x(itr_num);
    ground_msg.set_y(itr_num);
    ground_msg.set_z(itr_num);
    ground_msg.set_w(itr_num);

    publishMessageTest("/gazebo/default/test_pose_subscriber0_invalid", "gazebo_msgs/TestPose",
                       ground_msg, false);
}

/**
 * Invalid message type input for init publisher
 * Publish on not initialized topic of Gazebo
 * i.e. Try to publish on invalid message type
 */
TEST_F(pluginTestCustomHandler, testInvalidMessageTypeInitPublish) {
    resetScene();

    initPublisherTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose_Invalid",
                      false);

    stepScene(1);
}

/**
 * Invalid message type input for publish message
 * Publish on initialized topic of Gazebo
 * But error-out as message type is invalid
 */
TEST_F(pluginTestCustomHandler, testInvalidMessageTypePublish) {
    resetScene();

    // false as already initialized in testCustomMessagePublishSuccess TEST
    initPublisherTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose", true);

    stepScene(1);

    // Needed as take time to publish message from plugin side
    std::this_thread::sleep_for(std::chrono::microseconds(5000));

    TestPose ground_msg;
    double itr_num = 200.0;
    ground_msg.set_x(itr_num);
    ground_msg.set_y(itr_num);
    ground_msg.set_z(itr_num);
    ground_msg.set_w(itr_num);

    publishMessageTest("/gazebo/default/test_pose_subscriber0", "gazebo_msgs/TestPose_Invalid",
                       ground_msg, false);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
