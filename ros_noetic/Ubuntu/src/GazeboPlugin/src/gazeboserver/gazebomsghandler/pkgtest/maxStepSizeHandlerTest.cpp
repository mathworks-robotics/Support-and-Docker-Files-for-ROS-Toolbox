/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/MaxSimulationStepMsgHandler.hpp"

class maxStepSizeTest : public testing::Test {
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

        std::this_thread::sleep_for(std::chrono::microseconds(100000));

        /// Launch Max-Step-Size message handler
        std::thread Th0(
            &robotics::gazebotransport::GazeboServer::registerHandler, m_server,
            std::make_shared<robotics::gazebotransport::MaxSimulationStepMsgHandler>(m_world));

        Th0.join();

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    void TearDown() {
        /// Closes server and client
        m_client->shutdown();
        m_server->stop();
        gazebo::shutdown();
    }
};

/*
 * It tests success of the get max step size value from Gazebo
 * Also, it tests the client successfully gets the default step size (0.001)
 */
TEST_F(maxStepSizeTest, testGetStepSize) {
    /// Create Packet message to get max step size from Gazebo

    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_MAX_STEP_SIZE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_max_step_size()->set_type(
        mw::internal::robotics::gazebotransport::MaxStepSize_TYPE::MaxStepSize_TYPE_GET_STEP_SIZE);
    m_message.mutable_max_step_size()->set_size(0);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    ASSERT_EQ(reply.max_step_size().size(), 0.001); // default value 0.001
}

/*
 * It tests success of the set max step size value to Gazebo
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(maxStepSizeTest, testSetStepSize) {
    /// Create Packet message to set max step size of Gazebo
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_MAX_STEP_SIZE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_max_step_size()->set_type(
        mw::internal::robotics::gazebotransport::MaxStepSize_TYPE::MaxStepSize_TYPE_SET_STEP_SIZE);
    m_message.mutable_max_step_size()->set_size(0.01);

    /// Serialize, send and receives data
    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(time_out));

    /// Convert reply message to packet message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_NONE); // 0: NO Error
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
