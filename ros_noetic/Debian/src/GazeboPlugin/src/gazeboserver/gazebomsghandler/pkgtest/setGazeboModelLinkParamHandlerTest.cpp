/* Copyright 2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetGazeboModelParamMsgHandler.hpp"
#include "testUtils.hpp"

// Class to test set Gazebo model-link parameter
class setGazeboModelLinkParamTest
    : public ::testing::TestWithParam<
          std::tuple<std::string, std::string, modelLinkParamName, errorMsgValidation, bool>> {
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
 * It tests success of the set model-link parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_P(setGazeboModelLinkParamTest, testSetModelLinkParameters) {
    // run the world and reset model
    m_world->Run();
    m_world->Reset();

    // get testing parameters
    auto modelParam = GetParam();
    std::string modelName = std::get<0>(modelParam);
    std::string linkName = std::get<1>(modelParam);
    // type of test command
    modelLinkParamName testParam = std::get<2>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<3>(modelParam);
    // parameter status
    bool paramStatus = std::get<4>(modelParam);

    // ground truth
    double position_values[3] = {1.0, 2.0, 0.5};
    double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
    double prod_inertia_values[3] = {1.0, 2.0, 0.5};
    double pincipal_mom_values[3] = {1.0, 2.0, 0.5};
    double mass = 2.0;

    // send and receives reply for set model-link parameters
    mw::internal::robotics::gazebotransport::Packet reply =
        this->m_testUtils->clientSetModelLinkParam(
            modelName, linkName, position_values, orientation_values, prod_inertia_values,
            pincipal_mom_values, mass, paramStatus, testParam, this->m_client, this->time_out);

    // validate received error message
    m_testUtils->validateSetCommandErrorMsg(reply, error_type);

    // validate SET parameters with ground truth if there is no error
    if (error_type == Success) {
        m_testUtils->validateModelLinkResults(m_world, modelName, linkName, position_values,
                                              orientation_values, prod_inertia_values,
                                              pincipal_mom_values, mass, paramStatus, testParam);
    }
}

// test parameter initialization for get Gazebo model-link parameters
INSTANTIATE_TEST_CASE_P(
    testingSetModelLinkParameters,
    setGazeboModelLinkParamTest,
    ::testing::Values( // test model link pose
        std::make_tuple("2_body", "base", linkPose, Success, true),
        std::make_tuple("2_body_invalid", "base", linkPose, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkPose, InvalidLink, true),
        // test model link wind status
        std::make_tuple("2_body", "base", linkWind, Success, true),
        std::make_tuple("2_body", "base", linkWind, Success, false),
        std::make_tuple("2_body_invalid", "base", linkWind, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkWind, InvalidLink, true),
        // test model link self collide status
        std::make_tuple("2_body", "base", linkSelfCollide, Success, true),
        std::make_tuple("2_body", "base", linkSelfCollide, Success, false),
        std::make_tuple("2_body_invalid", "base", linkSelfCollide, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkSelfCollide, InvalidLink, true),
        // test model link isstatic status
        std::make_tuple("2_body", "base", linkIsStatic, Success, true),
        std::make_tuple("2_body", "base", linkIsStatic, Success, false),
        std::make_tuple("2_body_invalid", "base", linkIsStatic, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkIsStatic, InvalidLink, true),
        // test model link kinematics status
        std::make_tuple("2_body", "base", linkKinematics, Success, true),
        std::make_tuple("2_body", "base", linkKinematics, Success, false),
        std::make_tuple("2_body_invalid", "base", linkKinematics, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkKinematics, InvalidLink, true),
        // test model link gravity status
        std::make_tuple("2_body", "base", linkGravity, Success, true),
        std::make_tuple("2_body", "base", linkGravity, Success, false),
        std::make_tuple("2_body_invalid", "base", linkGravity, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkGravity, InvalidLink, true),
        // test model link canonical status
        std::make_tuple("2_body", "base", linkCanonical, Success, true),
        std::make_tuple("2_body", "base", linkCanonical, Success, false),
        std::make_tuple("2_body_invalid", "base", linkCanonical, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkCanonical, InvalidLink, true),
        // test model link mass
        std::make_tuple("2_body", "base", linkMass, Success, true),
        std::make_tuple("2_body_invalid", "base", linkMass, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkMass, InvalidLink, true),
        // test model link prod of inertia
        std::make_tuple("2_body", "base", linkProdOfInertia, Success, true),
        std::make_tuple("2_body_invalid", "base", linkProdOfInertia, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkProdOfInertia, InvalidLink, true),
        // test model link principal moment
        std::make_tuple("2_body", "base", linkPrincipalMom, Success, true),
        std::make_tuple("2_body_invalid", "base", linkPrincipalMom, InvalidModel, true),
        std::make_tuple("2_body", "base_invalid", linkPrincipalMom, InvalidLink, true)));

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
