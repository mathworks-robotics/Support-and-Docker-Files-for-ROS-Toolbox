/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetPoseMsgHandler.hpp"

class getPoseTest : public testing::Test {

  public:
    std::string ipAddress = "127.0.0.1";

    /// Session Time-out value
    int time_out = 1000;

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

    /**
     * It creates and sends get ground truth world pose message and receives the reply message from
     * server. Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientGetPose(std::string const& modelName,
                                                                  std::string const& linkName) {
        /// Create Packet message to get pose of a link
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_GET_GROUND_TRUTH_WORLD_POSE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_get_ground_truth_world_pose()->set_model_name(modelName);
        m_message.mutable_get_ground_truth_world_pose()->set_link_name(linkName);

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
        gazebo::physics::WorldPtr m_world = gazebo::loadWorld("world/unitBoxHandlerTest.world");
        if (!m_world) {
            // Could not load world
        }

        /// Launch GetPose message handler
        std::thread imageTh0(
            &robotics::gazebotransport::GazeboServer::registerHandler, m_server,
            std::make_shared<robotics::gazebotransport::GetPoseMsgHandler>(m_world));

        imageTh0.join();
    }

    void TearDown() {
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
    }
};

/*
 * It tests success of the get pose of a link
 * Also, it tests the client successfully gets pose of ground truth model/link
 */
TEST_F(getPoseTest, msgHandle1) {
    mw::internal::robotics::gazebotransport::Packet reply = clientGetPose("unit_box", "link");

    /// The ground truth position and orientation are defined in world/unit_box.world
    /// which is loaded at the start
    ASSERT_DOUBLE_EQ(reply.pose().position().x(), 2);
    ASSERT_DOUBLE_EQ(reply.pose().position().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().z(), 0.5);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().y(), 0.24740395925452296);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().w(), 0.96891242171064484);
}

/*
 * It tests reply message for invalid model name as input.
 * The client should successfully Pose message with all zero values
 */
TEST_F(getPoseTest, testInValidModelName) {
    mw::internal::robotics::gazebotransport::Packet reply = clientGetPose("unit_box0", "link");

    /// The input model is not available in Gazebo
    /// So it returns all zero values for position and orientation
    ASSERT_DOUBLE_EQ(reply.pose().position().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().w(), 0);
}

/*
 * It tests reply message for invalid link name as input.
 * The client should successfully Pose message with all zero values
 */
TEST_F(getPoseTest, testInValidLinkName) {
    mw::internal::robotics::gazebotransport::Packet reply = clientGetPose("unit_box", "link0");

    /// The input link is not available in Gazebo
    /// So it returns all zero values for position and orientation
    ASSERT_DOUBLE_EQ(reply.pose().position().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().position().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().x(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().y(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().z(), 0);
    ASSERT_DOUBLE_EQ(reply.pose().orientation().w(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
