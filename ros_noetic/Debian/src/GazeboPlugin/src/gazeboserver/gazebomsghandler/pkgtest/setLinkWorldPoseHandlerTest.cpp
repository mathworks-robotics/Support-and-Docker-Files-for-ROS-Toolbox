/* Copyright 2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetLinkWorldPoseMsgHandler.hpp"

class setLinkWorldPoseTest : public testing::Test {
  public:
    std::string ipAddress = "127.0.0.1";

    /// Session Time-out value
    int time_out = 1000;

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /// Gazebo message repository reference
    robotics::gazebotransport::GazeboApplyCommander m_commander;

    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_world;

    /// Gazebo connect a callback to the world update start signal
    gazebo::event::ConnectionPtr m_worldUpdateStartEventConnection;

    /**
     * It creates and sends apply link wrench message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientSetLinkWorldPose(
        std::string const& modelName,
        std::string const& linkName,
        double (&pose)[7],
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to apply force/torque on a link
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_SET_LINK_WORLD_POSE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_set_link_world_pose()->set_model_name(modelName);
        m_message.mutable_set_link_world_pose()->set_link_name(linkName);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_x(pose[0]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_y(pose[1]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_z(pose[2]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_x(
            pose[3]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_y(
            pose[4]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_z(
            pose[5]);
        m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_w(
            pose[6]);
        m_message.mutable_set_link_world_pose()->mutable_duration()->set_seconds(sec);
        m_message.mutable_set_link_world_pose()->mutable_duration()->set_nano_seconds(nsec);

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
        m_world = gazebo::loadWorld("world/applyForceTorqueNoPlugin.world");
        ASSERT_TRUE(m_world != nullptr) << "Fail to load the test scenario.";

        std::this_thread::sleep_for(std::chrono::microseconds(100000));

        /// Launch SetLinkWorldPoseMsgHandler message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::SetLinkWorldPoseMsgHandler>(m_world,
                                                                                    m_commander));

        // Starts thread for simulation begin update
        m_worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&setLinkWorldPoseTest::onWorldUpdateStart, this));

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    void onWorldUpdateStart() {
        m_commander.executeApplyCommands(m_world->SimTime());
    }

    void TearDown() {
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
    }
};

/*
 It tests success of the set world pose.
 Also, it tests the client successfully gets the NO Error message
*/
TEST_F(setLinkWorldPoseTest, testSetLinkWorldPoseSuccess) {
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    // ground truth
    double pose_values[7] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::string modelName = "unit_box";
    std::string linkName = "link";

    // retrieve model and link
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // set link world pose for 1 second
    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetLinkWorldPose(modelName, linkName, pose_values, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // step simulator for 1 second and check for link world pose
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[0], pose_values[0]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[1], pose_values[1]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[2], pose_values[2]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().X(), pose_values[3]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().Y(), pose_values[4]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().Z(), pose_values[5]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().W(), pose_values[6]);
}


/*
 It tests success of the set link world pose twice.
 Also, it tests the client successfully gets the NO Error message
*/
TEST_F(setLinkWorldPoseTest, testSetLinkWorldPoseTwiceSuccess) {
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    // ground truth
    double pose_values0[7] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::string modelName = "unit_box";
    std::string linkName = "link";

    // retrieve model and link
    auto model = m_world->ModelByName(modelName);
    auto link = model->GetLink(linkName);

    // set link world pose for 1 second
    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetLinkWorldPose(modelName, linkName, pose_values0, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // set world pose for 1 step (0.001 second)
    m_world->Step(1);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[0], pose_values0[0]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[1], pose_values0[1]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[2], pose_values0[2]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().X(), pose_values0[3]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().Y(), pose_values0[4]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().Z(), pose_values0[5]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().W(), pose_values0[6]);

    // change world pose values
    double pose_values1[7] = {3.0, 5.0, 5.0, 0.0, 0.0, 0.0, 1.0};

    // set link world pose for 1 second with new world pose
    reply = clientSetLinkWorldPose(modelName, linkName, pose_values1, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // set world pose for 1 step (0.001 second)
    m_world->Step(1);

    // validate updated world pose
    ignition::math::Pose3d _pose = link->WorldPose();
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[0], pose_values1[0]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[1], pose_values1[1]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Pos()[2], pose_values1[2]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().X(), pose_values1[3]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().Y(), pose_values1[4]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().Z(), pose_values1[5]);
    EXPECT_DOUBLE_EQ(link->WorldPose().Rot().W(), pose_values1[6]);
}


/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(setLinkWorldPoseTest, testInValidModelName) {
    double pose_values[7] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetLinkWorldPose("unt_box", "link", pose_values, 1, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID error
}

/*
 * It tests reply message for invalid link name as input.
 * The client should successfully receive the LINK_NAME_INVALID error message
 */
TEST_F(setLinkWorldPoseTest, testInValidLinkName) {
    double pose_values[7] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    mw::internal::robotics::gazebotransport::Packet reply =
        clientSetLinkWorldPose("unit_box", "lik", pose_values, 1, 0);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_LINK_NAME_INVALID); // LINK_NAME_INVALID
                                                        // error
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
