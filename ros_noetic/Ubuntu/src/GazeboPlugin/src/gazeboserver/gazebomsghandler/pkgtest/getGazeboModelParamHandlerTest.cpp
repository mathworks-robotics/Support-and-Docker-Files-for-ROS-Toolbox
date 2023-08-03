/* Copyright 2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/GetGazeboModelParamMsgHandler.hpp"
#include "testUtils.hpp"

// Class to test get Gazebo model, link and joint parameter
class getGazeboModelParamTest : public ::testing::TestWithParam<std::tuple<std::string,
                                                                           std::string,
                                                                           getModelParamOperation,
                                                                           errorMsgValidation,
                                                                           bool,
                                                                           modelJointParamName>> {
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
            std::make_shared<robotics::gazebotransport::GetGazeboModelParamMsgHandler>(m_world));

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
TEST_P(getGazeboModelParamTest, testGetModelParameters) {
    // pause the world and reset model
    m_world->Run();
    m_world->Reset();

    // get testing parameters
    auto modelParam = GetParam();
    std::string modelName = std::get<0>(modelParam);
    std::string linkJointName = std::get<1>(modelParam);
    // type of test command
    getModelParamOperation testParam = std::get<2>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<3>(modelParam);
    bool paramStatus = std::get<4>(modelParam);

    switch (testParam) {
    case GetModel: {
        // ground truth
        double position_values[3] = {1.0, 2.0, 0.5};
        double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};

        // set Gazebo model parameters with ground truth values
        if (error_type == Success) {
            m_testUtils->setGazeboModelParam(m_world, modelName, position_values,
                                             orientation_values, paramStatus);
        }

        // send and receives reply for get model parameters
        mw::internal::robotics::gazebotransport::Packet reply =
            this->m_testUtils->clientGetModelParam(modelName, this->m_client, this->time_out);

        // validate received reply with ground truth
        m_testUtils->validateGetGazeboModelParam(reply, modelName, position_values,
                                                 orientation_values, paramStatus, error_type);
    } break;

    case GetLink: {
        // ground truth
        double position_values[3] = {1.0, 2.0, 0.5};
        double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
        double prod_inertia_values[3] = {1.0, 2.0, 0.5};
        double pincipal_mom_values[3] = {1.0, 2.0, 0.5};
        double mass = 2.0;

        // set Gazebo model-link parameters with ground truth values
        if (error_type == Success) {
            m_testUtils->setGazeboModelLinkParam(m_world, modelName, linkJointName, position_values,
                                                 orientation_values, prod_inertia_values,
                                                 pincipal_mom_values, mass, paramStatus);
        }

        // send and receives reply for get model-link parameters
        mw::internal::robotics::gazebotransport::Packet reply =
            this->m_testUtils->clientGetModelLinkParam(modelName, linkJointName, this->m_client,
                                                       this->time_out);

        // validate received reply with ground truth
        m_testUtils->validateGetGazeboModelLinkParam(
            reply, modelName, linkJointName, position_values, orientation_values,
            prod_inertia_values, pincipal_mom_values, mass, paramStatus, paramStatus, paramStatus,
            paramStatus, paramStatus, paramStatus, error_type);
    } break;

    case GetJoint: {
        // ground truth
        double position_values[3] = {0.001, 0.002, 0.001};
        double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
        double xyz_values[3] = {0.0, 1.0, 0.0};
        double fudgeFactor_values = 1.0;
        double cfm_values = 1.0;
        double suspensionCfm_values = 0.0;
        double suspensionErp_values = 0.0;
        double angle_values = 0.5;
        double damping_values = 1.0;
        double friction_values = 1.0;
        uint32_t axis_index = static_cast<uint32_t>(std::get<4>(modelParam));
        modelJointParamName setParamName = std::get<5>(modelParam);

        // set Gazebo model-joint parameters with ground truth values
        if (error_type == Success) {
            m_testUtils->setGazeboModelJointParam(
                m_world, modelName, linkJointName, position_values, orientation_values, xyz_values,
                fudgeFactor_values, cfm_values, suspensionCfm_values, suspensionErp_values,
                axis_index, angle_values, damping_values, friction_values, setParamName);
        }

        // send and receives reply for get model-joint parameters
        mw::internal::robotics::gazebotransport::Packet reply =
            this->m_testUtils->clientGetModelJointParam(modelName, linkJointName, this->m_client,
                                                        this->time_out);

        // validate received reply with ground truth
        m_testUtils->validateGetGazeboModelJointParam(
            reply, modelName, linkJointName, position_values, orientation_values, xyz_values,
            fudgeFactor_values, cfm_values, suspensionCfm_values, suspensionErp_values, axis_index,
            angle_values, damping_values, friction_values, error_type, setParamName);
    } break;
    default:
        break;
    }
}

// test parameter initialization for set Gazebo model parameters
INSTANTIATE_TEST_CASE_P(
    testingGetModelParameters,
    getGazeboModelParamTest,
    ::testing::Values(
        // Get model param
        std::make_tuple("2_body", "", GetModel, Success, true, jointPose),
        std::make_tuple("2_body", "", GetModel, Success, false, jointPose),
        std::make_tuple("2_body_invalid", "", GetModel, InvalidModel, true, jointPose),
        // Get model link param
        std::make_tuple("2_body", "base", GetLink, Success, true, jointPose),
        std::make_tuple("2_body", "base", GetLink, Success, false, jointPose),
        std::make_tuple("2_body_invalid", "base", GetLink, InvalidModel, true, jointPose),
        std::make_tuple("2_body", "base_invalid", GetLink, InvalidLink, true, jointPose),
        // Get model joint param
        std::make_tuple("2_body", "follower_joint", GetJoint, Success, false, jointPose),
        std::make_tuple("2_body", "follower_joint", GetJoint, Success, false, jointXYZ),
        std::make_tuple("2_body", "follower_joint", GetJoint, Success, false, jointAngle),
        std::make_tuple("2_body", "follower_joint", GetJoint, Success, false, jointFudgeFac),
        std::make_tuple("2_body_invalid",
                        "follower_joint",
                        GetJoint,
                        InvalidModel,
                        false,
                        jointPose),
        std::make_tuple("2_body",
                        "follower_joint_invalid",
                        GetJoint,
                        InvalidJoint,
                        false,
                        jointPose),
        std::make_tuple("universal_joint", "universal_joint", GetJoint, Success, false, jointPose),
        std::make_tuple("universal_joint", "universal_joint", GetJoint, Success, true, jointPose),
        std::make_tuple("universal_joint", "universal_joint", GetJoint, Success, false, jointXYZ),
        std::make_tuple("universal_joint", "universal_joint", GetJoint, Success, true, jointXYZ),
        std::make_tuple("universal_joint", "universal_joint", GetJoint, Success, false, jointAngle),
        std::make_tuple("universal_joint", "universal_joint", GetJoint, Success, true, jointAngle),
        std::make_tuple("universal_joint",
                        "universal_joint",
                        GetJoint,
                        Success,
                        false,
                        jointFudgeFac),
        std::make_tuple("universal_joint",
                        "universal_joint",
                        GetJoint,
                        Success,
                        true,
                        jointFudgeFac),
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        GetJoint,
                        InvalidModel,
                        false,
                        jointPose),
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        GetJoint,
                        InvalidModel,
                        true,
                        jointPose),
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        GetJoint,
                        InvalidLink,
                        false,
                        jointPose),
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        GetJoint,
                        InvalidLink,
                        true,
                        jointPose)));

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
