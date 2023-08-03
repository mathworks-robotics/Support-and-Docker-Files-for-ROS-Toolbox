/* Copyright 2020 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/SetGazeboModelParamMsgHandler.hpp"
#include "testUtils.hpp"

// Class to test set Gazebo model-joint parameter
class setGazeboModelJointParamTest
    : public ::testing::TestWithParam<
          std::tuple<std::string, std::string, modelJointParamName, errorMsgValidation, uint32_t>> {
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
 * It tests success of the set model-joint parameter.
 * Also, it tests the client successfully receives without any Error message
 * Parameterized testing is used to test various combinations.
 */
TEST_P(setGazeboModelJointParamTest, testSetModelJointParameters) {
    // run the world and reset model
    m_world->Run();
    m_world->Reset();

    // get testing parameters
    auto modelParam = GetParam();
    std::string modelName = std::get<0>(modelParam);
    std::string jointName = std::get<1>(modelParam);
    // type of test command
    modelJointParamName testParam = std::get<2>(modelParam);
    // type of error message
    errorMsgValidation error_type = std::get<3>(modelParam);
    // axis index
    uint32_t axis_index = std::get<4>(modelParam);

    // ground truth
    double position_values[3] = {2.0, 2.0, 0.5};
    double orientation_values[4] = {1.0, 0.0, 0.0, 0.0};
    double xyz_values[3] = {0.0, 1.0, 0.0};
    double fudgeFactor_values = 1.0;
    double cfm_values = 1.0;
    double suspensionCfm_values = 0.5;
    double suspensionErp_values = 0.5;
    double angle_values = 1.0;
    double damping_values = 1.0;
    double friction_values = 1.0;

    // send and receives reply for set model-joint parameters
    mw::internal::robotics::gazebotransport::Packet reply =
        this->m_testUtils->clientSetModelJointParam(
            modelName, jointName, position_values, orientation_values, xyz_values,
            fudgeFactor_values, cfm_values, suspensionCfm_values, suspensionErp_values, axis_index,
            angle_values, damping_values, friction_values, testParam, this->m_client,
            this->time_out);

    // validate received error message
    m_testUtils->validateSetCommandErrorMsg(reply, error_type);

    // validate SET parameters with ground truth if there is no error
    if (error_type == Success) {
        m_testUtils->validateModelJointResults(
            m_world, modelName, jointName, position_values, orientation_values, xyz_values,
            fudgeFactor_values, cfm_values, suspensionCfm_values, suspensionErp_values, axis_index,
            angle_values, damping_values, friction_values, testParam);
    }
}

// test parameter initialization for get Gazebo model-joint pose parameters
INSTANTIATE_TEST_CASE_P(
    testingSetModelJointPoseParameters,
    setGazeboModelJointParamTest,
    ::testing::Values(
        // test model joint pose
        std::make_tuple("2_body", "base_joint", jointPose, Success, 0),                  // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointPose, InvalidModel, 0),     // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointPose, InvalidJoint, 0),     // No axis
        std::make_tuple("2_body", "follower_joint", jointPose, Success, 0),              // One axis
        std::make_tuple("2_body_invalid", "follower_joint", jointPose, InvalidModel, 0), // One axis
        std::make_tuple("2_body", "follower_joint_invalid", jointPose, InvalidJoint, 0), // One axis
        std::make_tuple("universal_joint", "universal_joint", jointPose, Success, 0),    // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointPose,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointPose,
                        InvalidJoint,
                        0),                                                           // Two axis
        std::make_tuple("universal_joint", "universal_joint", jointPose, Success, 1), // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointPose,
                        InvalidModel,
                        1), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointPose,
                        InvalidJoint,
                        1))); // Two axis

// test parameter initialization for get Gazebo model-joint parameters
INSTANTIATE_TEST_CASE_P(
    testingSetModelJointParameters,
    setGazeboModelJointParamTest,
    ::testing::Values(
        // test model joint cfm
        std::make_tuple("2_body", "base_joint", jointCfm, NoneAxis, 0),                 // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointCfm, InvalidModel, 0),     // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointCfm, InvalidJoint, 0),     // No axis
        std::make_tuple("2_body", "follower_joint", jointCfm, Success, 0),              // One axis
        std::make_tuple("2_body_invalid", "follower_joint", jointCfm, InvalidModel, 0), // One axis
        std::make_tuple("2_body", "follower_joint_invalid", jointCfm, InvalidJoint, 0), // One axis
        std::make_tuple("universal_joint", "universal_joint", jointCfm, Success, 0),    // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointCfm,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointCfm,
                        InvalidJoint,
                        0), // Two axis
        // test model joint fudge factor
        std::make_tuple("2_body", "base_joint", jointFudgeFac, NoneAxis, 0),             // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointFudgeFac, InvalidModel, 0), // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointFudgeFac, InvalidJoint, 0), // No axis
        std::make_tuple("2_body", "follower_joint", jointFudgeFac, Success, 0),          // One axis
        std::make_tuple("2_body_invalid", "follower_joint", jointFudgeFac, InvalidModel, 0), // One
                                                                                             // axis
        std::make_tuple("2_body", "follower_joint_invalid", jointFudgeFac, InvalidJoint, 0), // One
                                                                                             // axis
        std::make_tuple("universal_joint", "universal_joint", jointFudgeFac, Success, 0),    // Two
                                                                                             // axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointFudgeFac,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointFudgeFac,
                        InvalidJoint,
                        0), // Two axis
        // test model joint suspension cfm
        std::make_tuple("2_body", "base_joint", jointSupCfm, NoneAxis, 0),             // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointSupCfm, InvalidModel, 0), // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointSupCfm, InvalidJoint, 0), // No axis
        std::make_tuple("2_body_invalid", "follower_joint", jointSupCfm, InvalidModel, 0), // One
                                                                                           // axis
        std::make_tuple("2_body", "follower_joint_invalid", jointSupCfm, InvalidJoint, 0), // One
                                                                                           // axis
        // axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointSupCfm,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointSupCfm,
                        InvalidJoint,
                        0), // Two axis
        // test model joint suspension erp
        std::make_tuple("2_body", "base_joint", jointSupErp, NoneAxis, 0),             // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointSupErp, InvalidModel, 0), // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointSupErp, InvalidJoint, 0), // No axis
        std::make_tuple("2_body_invalid", "follower_joint", jointSupErp, InvalidModel, 0), // One
                                                                                           // axis
        std::make_tuple("2_body", "follower_joint_invalid", jointSupErp, InvalidJoint, 0), // One
                                                                                           // axis
        // axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointSupErp,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointSupErp,
                        InvalidJoint,
                        0))); // Two axis

// test parameter initialization for get Gazebo model-joint axis damping and friction parameters
INSTANTIATE_TEST_CASE_P(
    testingSetModelJointAxisParametersI,
    setGazeboModelJointParamTest,
    ::testing::Values(
        // test model joint axis damping
        std::make_tuple("2_body", "base_joint", jointDamping, NoneAxis, 0),             // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointDamping, InvalidModel, 0), // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointDamping, InvalidJoint, 0), // No axis
        std::make_tuple("2_body", "follower_joint", jointDamping, Success, 0),          // One axis
        std::make_tuple("2_body_invalid", "follower_joint", jointDamping, InvalidModel, 0), // One
                                                                                            // axis
        std::make_tuple("2_body", "follower_joint_invalid", jointDamping, InvalidJoint, 0), // One
                                                                                            // axis
        std::make_tuple("3_body", "follower_joint", jointDamping, Success, 0), // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointDamping,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointDamping,
                        InvalidJoint,
                        0),                                                              // Two axis
        std::make_tuple("universal_joint", "universal_joint", jointDamping, Success, 1), // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointDamping,
                        InvalidModel,
                        1), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointDamping,
                        InvalidJoint,
                        1), // Two axis
        // test model joint axis friction
        std::make_tuple("2_body", "base_joint", jointFriction, NoneAxis, 0),             // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointFriction, InvalidModel, 0), // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointFriction, InvalidJoint, 0), // No axis
        std::make_tuple("2_body", "follower_joint", jointFriction, Success, 0),          // One axis
        std::make_tuple("2_body_invalid", "follower_joint", jointFriction, InvalidModel, 0), // One
                                                                                             // axis
        std::make_tuple("2_body", "follower_joint_invalid", jointFriction, InvalidJoint, 0), // One
                                                                                             // axis
        std::make_tuple("universal_joint", "universal_joint", jointFriction, Success, 0),    // Two
                                                                                             // axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointFriction,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointFriction,
                        InvalidJoint,
                        0), // Two axis
        std::make_tuple("universal_joint", "universal_joint", jointFriction, Success, 1), // Two
                                                                                          // axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointFriction,
                        InvalidModel,
                        1), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointFriction,
                        InvalidJoint,
                        1))); // Two axis

// test parameter initialization for get Gazebo model-joint xyz and angle parameters
INSTANTIATE_TEST_CASE_P(
    testingSetModelJointAxisParametersII,
    setGazeboModelJointParamTest,
    ::testing::Values(
        // test model joint axis angle
        std::make_tuple("2_body", "base_joint", jointAngle, NoneAxis, 0),             // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointAngle, InvalidModel, 0), // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointAngle, InvalidJoint, 0), // No axis
        std::make_tuple("2_body", "follower_joint", jointAngle, Success, 0),          // One axis
        std::make_tuple("2_body_invalid", "follower_joint", jointAngle, InvalidModel, 0), // One
                                                                                          // axis
        std::make_tuple("2_body", "follower_joint_invalid", jointAngle, InvalidJoint, 0), // One
                                                                                          // axis
        std::make_tuple("universal_joint", "universal_joint", jointAngle, Success, 0), // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointAngle,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointAngle,
                        InvalidJoint,
                        0),                                                            // Two axis
        std::make_tuple("universal_joint", "universal_joint", jointAngle, Success, 1), // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointAngle,
                        InvalidModel,
                        1), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointAngle,
                        InvalidJoint,
                        1), // Two axis
        // test model joint axis xyz
        std::make_tuple("2_body", "base_joint", jointXYZ, NoneAxis, 0),                 // No axis
        std::make_tuple("2_body_invalid", "base_joint", jointXYZ, InvalidModel, 0),     // No axis
        std::make_tuple("2_body", "base_joint_invalid", jointXYZ, InvalidJoint, 0),     // No axis
        std::make_tuple("2_body", "follower_joint", jointXYZ, Success, 0),              // One axis
        std::make_tuple("2_body_invalid", "follower_joint", jointXYZ, InvalidModel, 0), // One axis
        std::make_tuple("2_body", "follower_joint_invalid", jointXYZ, InvalidJoint, 0), // One axis
        std::make_tuple("universal_joint", "universal_joint", jointXYZ, Success, 0),    // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointXYZ,
                        InvalidModel,
                        0), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointXYZ,
                        InvalidJoint,
                        0),                                                          // Two axis
        std::make_tuple("universal_joint", "universal_joint", jointXYZ, Success, 1), // Two axis
        std::make_tuple("universal_joint_invalid",
                        "universal_joint",
                        jointXYZ,
                        InvalidModel,
                        1), // Two axis
        std::make_tuple("universal_joint",
                        "universal_joint_invalid",
                        jointXYZ,
                        InvalidJoint,
                        1))); // Two axis

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
