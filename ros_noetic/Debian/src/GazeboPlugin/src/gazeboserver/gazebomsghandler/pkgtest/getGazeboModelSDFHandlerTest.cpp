/* Copyright 2021 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetGazeboModelSDFMsgHandler.hpp"
#include "testUtils.hpp"

// Class to test get Gazebo model, link and joint parameter
class getGazeboModelSDFTest : public testing::Test {
  public:
    std::string ipAddress = "127.0.0.1";
    /// Session Time-out value
    int time_out = 1000;

    /// Creating GazeboServer and Client Object
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;
    /// Test Utils object
    std::shared_ptr<testUtils> m_testUtils;
    /// Gazebo world pointer
    gazebo::physics::WorldPtr m_world;

    void SetUp() {
        /// Launch & Run GazeboServer
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();
        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(time_out));

        m_testUtils = std::make_shared<testUtils>();

        // Start Gazebo Simulator Server
        gazebo::setupServer();

        /// Load unit-box world file
        m_world = gazebo::loadWorld("world/setGetParamPluginTest.world");
        ASSERT_TRUE(m_world != nullptr) << "Fail to load the test scenario.";

        std::this_thread::sleep_for(std::chrono::microseconds(100000));

        /// Launch setJointPosition message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::GetGazeboModelSDFMsgHandler>(m_world));

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
 * It tests success of the get model, joint and link parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_F(getGazeboModelSDFTest, testGetModelSDFRead) {
    // pause the world and reset model
    m_world->Run();
    m_world->Reset();

    // get testing parameters
    std::string modelName = "2_body";
    std::string expectedModelName = modelName;
    bool isValid = true;
    // send and receives reply for get model parameters
    mw::internal::robotics::gazebotransport::Packet reply =
        this->m_testUtils->clientGetModelSDF(modelName, this->m_client, this->time_out);

    this->m_testUtils->verifyExtractedSDF(reply, expectedModelName, isValid);
}

TEST_F(getGazeboModelSDFTest, testGetInvalidModelSDFRead) {
    // pause the world and reset model
    m_world->Run();
    m_world->Reset();

    // get testing parameters
    std::string modelName = "invalid_2_body";
    std::string expectedModelName = "";
    bool isValid = false;
    // send and receives reply for get model parameters
    mw::internal::robotics::gazebotransport::Packet reply =
        this->m_testUtils->clientGetModelSDF(modelName, this->m_client, this->time_out);

    this->m_testUtils->verifyExtractedSDF(reply, expectedModelName, isValid);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
