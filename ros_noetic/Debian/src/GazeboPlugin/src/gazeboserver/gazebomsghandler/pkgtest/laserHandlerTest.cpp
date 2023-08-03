/* Copyright 2019-2022 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetLaserMsgHandler.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SubscribeLaserMsgHandler.hpp"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/gazebo.hh"

class laserPublish : public testing::Test {

  public:
    std::string ipAddress = "127.0.0.1";

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /// Store Gazebo Simulation time
    gazebo::msgs::Time simTime;

    /// Threads to publish laser sensor 0 and laser sensor 1 data
    std::shared_ptr<std::thread> th0;
    std::shared_ptr<std::thread> th1;

    /// Session Time-out value
    int time_out = 5000;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr world;

    // Gazebo Node pointer
    gazebo::transport::NodePtr nodeSub;

    /// Ground truth values published on laser sensor 0 and sensor 1
    double gndlaserValues0[9] = {-3.14, 3.14, 0.0098278560250391247, 640, 0.0, 0.0, 0.0, 0.08, 10};
    double gndlaserValues1[9] = {-1.14, 1.14, 0.0215452475222447522, 240, 0.0, 0.0, 0.0, 0.07, 5};

    /// Keep publisher threads (th0 & th1 ) alive
    int clientEnd = 1;

    // Indicates publishing of laser sensor 0 and laser sensor 1 is started
    bool startedPublaser0 = false;
    bool startedPublaser1 = false;

    /**
     * @param set_frame_name       Gazebo sensor frame name
     * @param gndlaserValues       Ground truth values
     * @param pose                 Position of sensor in Gazebo
     *
     * This function creates LaserStamped data, which is used by laser sensor publisher
     */
    gazebo::msgs::LaserScanStamped getLaserMessage(std::string const& set_frame_name,
                                                   double (&gndlaserValues)[9],
                                                   ignition::math::Pose3d& pose) {
        // creates laser message
        gazebo::msgs::LaserScanStamped laserMsg;
        gazebo::common::Time timestamp = world->SimTime();
        gazebo::msgs::Set(laserMsg.mutable_time(), timestamp);

        laserMsg.mutable_scan()->set_frame(set_frame_name);

        gazebo::msgs::Set(laserMsg.mutable_scan()->mutable_world_pose(), pose);
        laserMsg.mutable_scan()->set_angle_min(gndlaserValues[0]);
        laserMsg.mutable_scan()->set_angle_max(gndlaserValues[1]);
        laserMsg.mutable_scan()->set_angle_step(gndlaserValues[2]);
        laserMsg.mutable_scan()->set_count(gndlaserValues[3]);

        laserMsg.mutable_scan()->set_vertical_angle_min(gndlaserValues[4]);
        laserMsg.mutable_scan()->set_vertical_angle_max(gndlaserValues[5]);
        laserMsg.mutable_scan()->set_vertical_angle_step(gndlaserValues[6]);
        laserMsg.mutable_scan()->set_vertical_count(1);

        laserMsg.mutable_scan()->set_range_min(gndlaserValues[7]);
        laserMsg.mutable_scan()->set_range_max(gndlaserValues[8]);

        // Adds intensity and range values
        if (laserMsg.mutable_scan()->ranges_size() != gndlaserValues[3]) {
            laserMsg.mutable_scan()->clear_ranges();
            laserMsg.mutable_scan()->clear_intensities();
            for (int i = 0; i < gndlaserValues[3]; ++i) {
                laserMsg.mutable_scan()->add_ranges(ignition::math::NAN_F);
                laserMsg.mutable_scan()->add_intensities(ignition::math::NAN_F);
            }
        }

        // Sets intensity and range values
        for (int i = 0; i < gndlaserValues[3]; i++) {
            double range = ignition::math::INF_D;
            laserMsg.mutable_scan()->set_ranges(i, range);
            laserMsg.mutable_scan()->set_intensities(i, 0);
        }

        return laserMsg;
    }

    /**
     * It publishes laser sensor zero data. The topic name is initialized inside this function.
     */
    void laserSensorZeroPublisher() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::LaserScanStamped>(
            "/gazebo/default/hokuyo0/link/laser/scan");

        // Pose of laser sensor 0
        ignition::math::Pose3d pose(0.01, 0, 0.03, 0, 0, 0);
        /// Publish laser sensor 0 data in a loop
        while (this->clientEnd) {
            // Get & publish LaserScanStamped data
            gazebo::msgs::LaserScanStamped laserMsg =
                getLaserMessage("hokuyo0::link", gndlaserValues0, pose);
            pub->Publish(laserMsg);
            this->startedPublaser0 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }

    /**
     * It publish laser sensor one data. The topic name is initialized inside this function.
     */
    void laserSensorOnePublisher() {
        /// Initialize gazebo node
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        /// Start gazebo publisher
        gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::LaserScanStamped>(
            "/gazebo/default/hokuyo1/link/laser/scan");

        ignition::math::Pose3d pose(0.01, 0, 0.03, 0, 0, 0);
        /// Publish laser sensor 1 data in a loop
        while (this->clientEnd) {
            // Get & publish LaserScanStamped data
            gazebo::msgs::LaserScanStamped laserMsg =
                getLaserMessage("hokuyo1::link", gndlaserValues1, pose);
            pub->Publish(laserMsg);
            this->startedPublaser1 = true;
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }

    /**
     * It creates and sends subscribe laser message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSubscribeLaser(
        std::string const& topic_name) {
        /// Create Packet message to subscribe laser
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SUBSCRIBE_LASER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_subscribe_laser()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet replyGetLaser;
        if (replyMsg) {
            replyGetLaser.ParseFromString(*replyMsg);
        }

        return replyGetLaser;
    }

    /**
     * It creates and sends get/request laser message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetLaser(std::string const& topic_name) {
        /// Create Packet message to get laser
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_REQUEST_LASER);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_request_laser()->set_topic_name(topic_name);

        /// Serialize, send and receives data
        auto msg = m_message.SerializeAsString();
        auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

        /// Convert reply message to packet message
        mw::internal::robotics::gazebotransport::Packet replyGetLaser;
        if (replyMsg) {
            replyGetLaser.ParseFromString(*replyMsg);
        }

        return replyGetLaser;
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

        /// Launch LASER data publisher
        th0 = std::make_shared<std::thread>(&laserPublish::laserSensorZeroPublisher, this);
        th1 = std::make_shared<std::thread>(&laserPublish::laserSensorOnePublisher, this);

        // Ensure the laser pub is started
        while (!this->startedPublaser0 || !this->startedPublaser1) {
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }

        /// Init Node, SubPtr and GazeboMsgRepo for msgHandler
        nodeSub = gazebo::transport::NodePtr(new gazebo::transport::Node());
        nodeSub->Init();

        /// Message handler starts
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::SubscribeLaserMsgHandler>(world, nodeSub));
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::GetLaserMsgHandler>(world));

        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }

    void TearDown() {
        /// Close threads
        this->clientEnd = 0;
        th0->join();
        th1->join();
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
};

/*
 * Subscribe and Get Laser data of sensor 0
 * It tests the laser sensor 0 is successfully subscribed or not.
 * Also, it tests the client successfully gets the laser data or not.
 * The verifications are done with ground truth values.
 */
TEST_F(laserPublish, testLaserSensorZero) {
    world->Run();
    world->SetPaused(true);
    world->ResetTime();

    //*********************************************
    /// TEST Laser0 Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeLaser("/gazebo/default/hokuyo0/link/laser/scan");

    bool m_success0 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success0 = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success0);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    //*********************************************
    /// TEST Get Laser0 for first time
    mw::internal::robotics::gazebotransport::Packet replyGetLaser0 =
        clientGetLaser("/gazebo/default/hokuyo0/link/laser/scan");

    /// Verify the received laser data with ground truth values.
    ASSERT_EQ(replyGetLaser0.laser_data().angle_min(), gndlaserValues0[0]);
    ASSERT_EQ(replyGetLaser0.laser_data().angle_max(), gndlaserValues0[1]);
    ASSERT_EQ(replyGetLaser0.laser_data().angle_step(), gndlaserValues0[2]);
    ASSERT_EQ(replyGetLaser0.laser_data().count(), gndlaserValues0[3]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_min(), gndlaserValues0[4]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_max(), gndlaserValues0[5]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_step(), gndlaserValues0[6]);
    ASSERT_EQ(replyGetLaser0.laser_data().range_min(), gndlaserValues0[7]);
    ASSERT_EQ(replyGetLaser0.laser_data().range_max(), gndlaserValues0[8]);

    for (int i = 0; i < replyGetLaser0.laser_data().count(); i++) {
        ASSERT_EQ(replyGetLaser0.laser_data().range(0), 0);
        ASSERT_EQ(replyGetLaser0.laser_data().intensities(0), 0);
    }

    //*********************************************
    /// TEST Get Laser0 again without stepping
    /// i.e. laser message contains old update
    replyGetLaser0 = clientGetLaser("/gazebo/default/hokuyo0/link/laser/scan");

    /// The valid message send by Laser Sensor handler,
    // as there is no stepping and same topic called again.
    ASSERT_TRUE(replyGetLaser0.has_laser_data());

    /// Verify the received laser data with ground truth values.
    ASSERT_EQ(replyGetLaser0.laser_data().angle_min(), gndlaserValues0[0]);
    ASSERT_EQ(replyGetLaser0.laser_data().angle_max(), gndlaserValues0[1]);
    ASSERT_EQ(replyGetLaser0.laser_data().angle_step(), gndlaserValues0[2]);
    ASSERT_EQ(replyGetLaser0.laser_data().count(), gndlaserValues0[3]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_min(), gndlaserValues0[4]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_max(), gndlaserValues0[5]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_step(), gndlaserValues0[6]);
    ASSERT_EQ(replyGetLaser0.laser_data().range_min(), gndlaserValues0[7]);
    ASSERT_EQ(replyGetLaser0.laser_data().range_max(), gndlaserValues0[8]);

    //*********************************************
    /// TEST Get Laser message again with stepping
    /// i.e. Laser message contains new update

    // step and get Laser message
    world->Step(100);

    replyGetLaser0 = clientGetLaser("/gazebo/default/hokuyo0/link/laser/scan");

    /// Verify the received laser data with ground truth values.
    ASSERT_EQ(replyGetLaser0.laser_data().angle_min(), gndlaserValues0[0]);
    ASSERT_EQ(replyGetLaser0.laser_data().angle_max(), gndlaserValues0[1]);
    ASSERT_EQ(replyGetLaser0.laser_data().angle_step(), gndlaserValues0[2]);
    ASSERT_EQ(replyGetLaser0.laser_data().count(), gndlaserValues0[3]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_min(), gndlaserValues0[4]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_max(), gndlaserValues0[5]);
    ASSERT_EQ(replyGetLaser0.laser_data().vertical_angle_step(), gndlaserValues0[6]);
    ASSERT_EQ(replyGetLaser0.laser_data().range_min(), gndlaserValues0[7]);
    ASSERT_EQ(replyGetLaser0.laser_data().range_max(), gndlaserValues0[8]);

    for (int i = 0; i < replyGetLaser0.laser_data().count(); i++) {
        ASSERT_EQ(replyGetLaser0.laser_data().range(0), 0);
        ASSERT_EQ(replyGetLaser0.laser_data().intensities(0), 0);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

/*
 * It tests, if the laser sensor( topic) is not available in Gazebo
 * & user subscribing that topic. Then, it should return FALSE.
 */
TEST_F(laserPublish, testInValidTopic) {
    //*********************************************
    /// TEST Wrong Laser Subscriber

    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeLaser("/gazebo/default/hokuyo/link/laser/scan");

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
 * It tests, if the laser sensor( topic) is not available in Gazebo
 * & user trying to get laser data from that topic.
 * Then, all laser sensor message fields should zero.
 */
TEST_F(laserPublish, testGetInValidLaser) {
    //*********************************************
    /// TEST Get Invalid Laser

    mw::internal::robotics::gazebotransport::Packet replyGetWrongLaser =
        clientGetLaser("/gazebo/default/hokuyo/link/laser/scan");

    bool m_success0 = false;

    if (replyGetWrongLaser.has_status() && !replyGetWrongLaser.status()) {
        m_success0 = true;
    }
    // it should be false as topic is not available in gazebo
    ASSERT_FALSE(m_success0);
    ASSERT_EQ(replyGetWrongLaser.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_TOPIC_NAME_INVALID);
}

/*
 * Subscribe and Get Laser data of sensor 1
 * It tests the laser sensor 1 is successfully subscribed or not.
 * Also, it tests the client successfully gets the laser data or not.
 * The verifications are done with ground truth values.
 */
TEST_F(laserPublish, testLaserSensorOne) {
    //*********************************************
    /// TEST Laser1 Subscriber
    mw::internal::robotics::gazebotransport::Packet reply0 =
        clientSubscribeLaser("/gazebo/default/hokuyo1/link/laser/scan");

    bool m_success1 = false;

    if (reply0.has_status() && !reply0.status()) {
        m_success1 = true;
    }
    // it should be true as topic is available in gazebo
    ASSERT_TRUE(m_success1);

    std::this_thread::sleep_for(std::chrono::microseconds(100000));

    //*********************************************
    /// TEST Get Laser1
    mw::internal::robotics::gazebotransport::Packet replyGetLaser1 =
        clientGetLaser("/gazebo/default/hokuyo1/link/laser/scan");

    /// Verify the received laser data with ground truth values.
    ASSERT_EQ(replyGetLaser1.laser_data().angle_min(), gndlaserValues1[0]);
    ASSERT_EQ(replyGetLaser1.laser_data().angle_max(), gndlaserValues1[1]);
    ASSERT_EQ(replyGetLaser1.laser_data().angle_step(), gndlaserValues1[2]);
    ASSERT_EQ(replyGetLaser1.laser_data().count(), gndlaserValues1[3]);
    ASSERT_EQ(replyGetLaser1.laser_data().vertical_angle_min(), gndlaserValues1[4]);
    ASSERT_EQ(replyGetLaser1.laser_data().vertical_angle_max(), gndlaserValues1[5]);
    ASSERT_EQ(replyGetLaser1.laser_data().vertical_angle_step(), gndlaserValues1[6]);
    ASSERT_EQ(replyGetLaser1.laser_data().range_min(), gndlaserValues1[7]);
    ASSERT_EQ(replyGetLaser1.laser_data().range_max(), gndlaserValues1[8]);

    for (int i = 0; i < replyGetLaser1.laser_data().count(); i++) {
        ASSERT_EQ(replyGetLaser1.laser_data().range(0), 0);
        ASSERT_EQ(replyGetLaser1.laser_data().intensities(0), 0);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100000));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
