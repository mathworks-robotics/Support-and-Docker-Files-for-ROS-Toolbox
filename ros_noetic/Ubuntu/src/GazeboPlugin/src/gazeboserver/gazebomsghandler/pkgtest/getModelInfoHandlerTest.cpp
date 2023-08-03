/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetModelInfoMsgHandler.hpp"

class getModelInfoTest : public testing::Test {
  public:
    std::string ipAddress = "127.0.0.1";

    /// Session Time-out value
    int time_out = 1000;

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;

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

        /// Launch GetModelInfo message handler
        std::thread Th0(
            &robotics::gazebotransport::GazeboServer::registerHandler, m_server,
            std::make_shared<robotics::gazebotransport::GetModelInfoMsgHandler>(m_world));

        Th0.join();
    }

    void TearDown() {
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
    }
};

/*
 * It tests success of the get all Model information from Gazebo
 * Also, it tests the client successfully receives Model/link/joint names
 * by comparing with default as well as world file Model/link/joint names
 */
TEST_F(getModelInfoTest, testModelNames) {
    /// Create Packet message to get Model info from Gazebo
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_MODEL_INFO);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_model_info()->set_topic_name("~/GetModelInfo");

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    /// The received model info is compared with default gazebo model like "ground_plane" & link
    /// "link" Also, the ground truth model, link and joint names are given with .world file as
    /// "unit_box", "link" & "joint"
    ASSERT_STREQ(reply.model_info().model_data(0).model_name().c_str(), "ground_plane");
    ASSERT_STREQ(reply.model_info().model_data(0).links().link_name(0).c_str(), "link");
    ASSERT_STREQ(reply.model_info().model_data(1).model_name().c_str(), "unit_box");
    ASSERT_STREQ(reply.model_info().model_data(1).links().link_name(0).c_str(), "link");
    ASSERT_STREQ(reply.model_info().model_data(1).joints().joint_name(0).c_str(), "joint");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
