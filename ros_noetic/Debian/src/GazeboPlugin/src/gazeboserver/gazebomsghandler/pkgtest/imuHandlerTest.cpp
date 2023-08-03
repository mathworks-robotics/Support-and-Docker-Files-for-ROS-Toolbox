/* Copyright 2019-2022 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetImuMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeImuMsgHandler.hpp"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/gazebo.hh"

class imuPublish : public testing::Test {

  public:
    std::string ipAddress = "127.0.0.1";

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /// Store Gazebo Simulation time
    gazebo::msgs::Time simTime;

    /// Threads to publish imu sensor 0 and imu sensor 1 data
    std::shared_ptr<std::thread> th0;
    std::shared_ptr<std::thread> th1;

    /// Session Time-out value
    int time_out = 5000;

    // Gazebo Node pointer
    gazebo::transport::NodePtr nodeSub;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr world;

    /// Ground truth values published on imu sensor 0 and sensor 1
    double gndImuValues0[10] = {2.032, 1.251,   25.0254, 0.02025, 0.0225,
                                1.02,  0.00124, 0.01422, 1.02522, 0.00008};
    double gndImuValues1[10] = {9.24215, 2.0124, 0.02025, 5.0254, 1.252,
                                8.2512,  10.152, 2.0214,  9.2452, 1.02474};

    /// Keep publisher threads (th0 & th1 ) alive
    int clientEnd = 1;

    // Indicates publishing of imu sensor 0 and imu sensor 1 is started
    bool startedPubImu0 = false;
    bool startedPubImu1 = false;

    /**
     * @param set_frame_name       Gazebo sensor frame name
     * @param gndimuValues         Ground truth values
     *
     * This function creates IMU data, which is used by imu sensor publisher
     */
    gazebo::msgs::IMU getImuMessage(std::string const& set_frame_name, double (&gndimuValues)[10]) {
        // creates imu message
        gazebo::msgs::IMU imuMsg;

        ignition::math::Vector3d angularVel(gndimuValues[0], gndimuValues[1], gndimuValues[2]);
        ignition::math::Vector3d linearAccl(gndimuValues[3], gndimuValues[4], gndimuValues[5]);
        ignition::math::Quaterniond orientationVal(gndimuValues[9], gndimuValues[6],
                                                   gndimuValues[7], gndimuValues[8]);

        gazebo::common::Time timestamp = world->SimTime();
        gazebo::msgs::Set(imuMsg.mutable_stamp(), timestamp);

        imuMsg.set_entity_name(set_frame_name);
        gazebo::msgs::Set(imuMsg.mutable_angular_velocity(), angularVel);
        gazebo::msgs::Set(imuMsg.mutable_linear_acceleration(), linearAccl);
        gazebo::msgs::Set(imuMsg.mutable_orientation(), orientationVal);

        return imuMsg;
    }

    /**
     * It publishes imu sensor zero data. The topic name is initialized inside this function.
     */
    void imuSensorZeroPublisher() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub =
            node->Advertise<gazebo::msgs::IMU>("/gazebo/default/imu0/link/imu/imu", 200);

        /// Publish imu sensor 0 data in a loop
        while (this->clientEnd) {
            // Get & publish IMU data
            gazebo::msgs::IMU imuMsg = getImuMessage("imu0::link", gndImuValues0);
            pub->Publish(imuMsg);
            this->startedPubImu0 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }

    /**
     * It publish imu sensor one data. The topic name is initialized inside this function.
     */
    void imuSensorOnePublisher() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub =
            node->Advertise<gazebo::msgs::IMU>("/gazebo/default/imu1/link/imu/imu", 200);

        /// Publish imu sensor 1 data in a loop
        while (this->clientEnd) {
            // Get & publish IMU data
            gazebo::msgs::IMU imuMsg = getImuMessage("imu1::link", gndImuValues1);
            pub->Publish(imuMsg);
            this->startedPubImu1 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }
    /**
     * It creates and sends subscribe imu message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSubscribeImu(
        std::string const& topic_name) {
        /// Create Packet message to subscribe imu
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SUBSCRIBE_IMU);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_imu()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
    }

    /**
     * It creates and sends get/request imu message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetImu(std::string const& topic_name) {
        /// Create Packet message to get imu
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                               PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_IMU);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_imu()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet reply;
        if (replyMsg) {
            reply.ParseFromString(*replyMsg);
        }

        return reply;
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

        /// Launch IMU data publisher
        th0 = std::make_shared<std::thread>(&imuPublish::imuSensorZeroPublisher, this);
        th1 = std::make_shared<std::thread>(&imuPublish::imuSensorOnePublisher, this);

        // Ensure the imu pub is started
        while (!this->startedPubImu0 || !this->startedPubImu1) {
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }

        /// Init Node, SubPtr and GazeboMsgRepo for msgHandler
        nodeSub = gazebo::transport::NodePtr(new gazebo::transport::Node());
        nodeSub->Init();

        /// Message handler starts
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::SubscribeImuMsgHandler>(world, nodeSub));
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::GetImuMsgHandler>(world));

        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    void TearDown() {
        //*********************************************
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
};

/*
 * Subscribe and Get Imu data of sensor 0
 * It tests the imu sensor 0 is successfully subscribed or not.
 * Also, it tests the client successfully gets the imu data or not.
 * The verifications are done with ground truth values.
 */
TEST_F(imuPublish, testImuSensorZero) {
    world->Run();
    world->SetPaused(true);
    world->ResetTime();

    //*********************************************
    /// TEST Imu0 Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImu("/gazebo/default/imu0/link/imu/imu");

    bool m_success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success0 = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success0);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    //*********************************************
    /// TEST Get Imu0 for first time
    mw::internal::robotics::gazebotransport::Packet replyGetImu0 =
        clientGetImu("/gazebo/default/imu0/link/imu/imu");

    /// Verify the received imu data with ground truth values.
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().x(), gndImuValues0[0]);
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().y(), gndImuValues0[1]);
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().z(), gndImuValues0[2]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().x(), gndImuValues0[3]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().y(), gndImuValues0[4]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().z(), gndImuValues0[5]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().x(), gndImuValues0[6]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().y(), gndImuValues0[7]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().z(), gndImuValues0[8]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().w(), gndImuValues0[9]);

    //*********************************************
    /// TEST Get Imu0 again without stepping
    /// i.e. IMU message contains old update
    replyGetImu0 = clientGetImu("/gazebo/default/imu0/link/imu/imu");

    // The valid message send by IMU Sensor handler,
    // as there is no stepping and same topic called again.
    ASSERT_TRUE(replyGetImu0.has_imu_data());

    /// Verify the received imu data with ground truth values.
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().x(), gndImuValues0[0]);
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().y(), gndImuValues0[1]);
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().z(), gndImuValues0[2]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().x(), gndImuValues0[3]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().y(), gndImuValues0[4]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().z(), gndImuValues0[5]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().x(), gndImuValues0[6]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().y(), gndImuValues0[7]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().z(), gndImuValues0[8]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().w(), gndImuValues0[9]);

    //*********************************************
    /// TEST Get IMU message again with stepping
    /// i.e. IMU message contains new update

    // step and get IMU message
    world->Step(100);
    replyGetImu0 = clientGetImu("/gazebo/default/imu0/link/imu/imu");

    /// Verify the received imu data with ground truth values.
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().x(), gndImuValues0[0]);
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().y(), gndImuValues0[1]);
    ASSERT_EQ(replyGetImu0.imu_data().angular_velocity().z(), gndImuValues0[2]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().x(), gndImuValues0[3]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().y(), gndImuValues0[4]);
    ASSERT_EQ(replyGetImu0.imu_data().linear_acceleration().z(), gndImuValues0[5]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().x(), gndImuValues0[6]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().y(), gndImuValues0[7]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().z(), gndImuValues0[8]);
    ASSERT_EQ(replyGetImu0.imu_data().orientation().w(), gndImuValues0[9]);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

/*
 * It tests, if the imu sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(imuPublish, testInValidTopic) {
    //*********************************************
    /// TEST Wrong Imu Subscriber

    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImu("/gazebo/default/imu/link/imu/imu");

    bool m_success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(m_success0);
    ASSERT_EQ(reply0.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                   Packet_CoSimError_TOPIC_NAME_INVALID);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

/*
 * It tests, if the imu sensor( topic) is not available in Gazebo
 * & user trying to get imu data from that topic.
 * Then, all imu sensor message fields should zero.
 */
TEST_F(imuPublish, testGetInValidImu) {
    //*********************************************
    /// TEST Get Invalid Imu

    mw::internal::robotics::gazebotransport::Packet replyGetWrongImu =
        clientGetImu("/gazebo/default/imu/link/imu/imu");

    bool m_success0 = false;

    if (replyGetWrongImu.has_status() && !replyGetWrongImu.status()) {
        m_success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(m_success0);
    ASSERT_EQ(replyGetWrongImu.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TOPIC_NAME_INVALID);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

/*
 * Subscribe and Get Imu data of sensor 1
 * It tests the Imu sensor 1 is successfully subscribed or not.
 * Also, it tests the client successfully gets the Imu data or not.
 * The verifications are done with ground truth values.
 */
TEST_F(imuPublish, testImuSensorOne) {
    //*********************************************
    /// TEST imu1 Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeImu("/gazebo/default/imu1/link/imu/imu");

    bool m_success1 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success1 = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success1);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    //*********************************************
    /// TEST Get imu1
    mw::internal::robotics::gazebotransport::Packet replyGetImu1 =
        clientGetImu("/gazebo/default/imu1/link/imu/imu");

    /// Verify the received imu data with ground truth values.
    ASSERT_EQ(replyGetImu1.imu_data().angular_velocity().x(), gndImuValues1[0]);
    ASSERT_EQ(replyGetImu1.imu_data().angular_velocity().y(), gndImuValues1[1]);
    ASSERT_EQ(replyGetImu1.imu_data().angular_velocity().z(), gndImuValues1[2]);
    ASSERT_EQ(replyGetImu1.imu_data().linear_acceleration().x(), gndImuValues1[3]);
    ASSERT_EQ(replyGetImu1.imu_data().linear_acceleration().y(), gndImuValues1[4]);
    ASSERT_EQ(replyGetImu1.imu_data().linear_acceleration().z(), gndImuValues1[5]);
    ASSERT_EQ(replyGetImu1.imu_data().orientation().x(), gndImuValues1[6]);
    ASSERT_EQ(replyGetImu1.imu_data().orientation().y(), gndImuValues1[7]);
    ASSERT_EQ(replyGetImu1.imu_data().orientation().z(), gndImuValues1[8]);
    ASSERT_EQ(replyGetImu1.imu_data().orientation().w(), gndImuValues1[9]);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
