/* Copyright 2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetGazeboModelParamMsgHandler.hpp"
#include "testUtils.hpp"

// Class to test set Gazebo model parameter
class setGazeboModelParamTest
    : public ::testing::TestWithParam<
          std::tuple<std::string, std::string, modelParamName, errorMsgValidation, bool>> {
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
            std::make_shared<robotics::gazebotransport::SetGazeboModelParamMsgHandler>(m_world));

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
 * It tests success of the set model parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_P(setGazeboModelParamTest, testSetModelParameters) {
    // run the world and reset model
    m_world->Run();
    m_world->Reset();

    // get testing parameters
    auto modelParam = GetParam();
    std::string modelName = std::get<0>(modelParam);
    // type of test command
    modelParamName testParam = std::get<2>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<3>(modelParam);
    // parameter status
    bool paramStatus = std::get<4>(modelParam);

    // ground truth
    double position_values[3] = {1.0, 2.0, 0.5};
    double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};

    // send and receives reply for set model parameters
    mw::internal::robotics::gazebotransport::Packet reply = this->m_testUtils->clientSetModelParam(
        modelName, position_values, orientation_values, paramStatus, testParam, this->m_client,
        this->time_out);

    // validate received error message
    m_testUtils->validateSetCommandErrorMsg(reply, error_type);

    // validate SET parameters with ground truth if there is no error
    if (error_type == Success) {
        m_testUtils->validateModelResults(m_world, modelName, position_values, orientation_values,
                                          paramStatus, testParam);
    }
}

// test parameter initialization for get Gazebo model parameters
INSTANTIATE_TEST_CASE_P(
    testingSetModelParameters,
    setGazeboModelParamTest,
    ::testing::Values(std::make_tuple("2_body", "", Pose, Success, true),
                      std::make_tuple("2_body_invalid", "", Pose, InvalidModel, true),
                      std::make_tuple("2_body", "", Wind, Success, true),
                      std::make_tuple("2_body", "", Wind, Success, false),
                      std::make_tuple("2_body_invalid", "", Wind, InvalidModel, true),
                      std::make_tuple("2_body", "", SelfCollide, Success, true),
                      std::make_tuple("2_body", "", SelfCollide, Success, false),
                      std::make_tuple("2_body_invalid", "", SelfCollide, InvalidModel, true),
                      std::make_tuple("2_body", "", IsStatic, Success, true),
                      std::make_tuple("2_body", "", IsStatic, Success, false),
                      std::make_tuple("2_body_invalid", "", IsStatic, InvalidModel, true)));

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
