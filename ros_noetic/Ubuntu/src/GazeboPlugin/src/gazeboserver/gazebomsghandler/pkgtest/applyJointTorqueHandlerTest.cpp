/* Copyright 2019 The MathWorks, Inc. */
#include <gtest/gtest.h>
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "gazebotransport/Server.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/ApplyJointTorqueMsgHandler.hpp"

class applyJointTorqueTest : public testing::Test {
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
     * It creates and sends apply joint torque message and receives the reply message from server.
     * Further, it creates and returns Packet message from reply message.
     */
    mw::internal::robotics::gazebotransport::Packet clientApplyJointTorque(
        std::string const& modelName,
        std::string const& jointName,
        uint32_t indexVal,
        double effortVal,
        uint64_t sec,
        uint64_t nsec) {
        /// Create Packet message to apply torque on a joint
        mw::internal::robotics::gazebotransport::Packet m_message;
        m_message.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                PacketHeader_MsgID_APPLY_JOINT_TORQUE);
        m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
        m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        m_message.mutable_apply_joint_torque()->set_model_name(modelName);
        m_message.mutable_apply_joint_torque()->set_joint_name(jointName);
        m_message.mutable_apply_joint_torque()->set_index(indexVal);
        m_message.mutable_apply_joint_torque()->set_effort(effortVal);
        m_message.mutable_apply_joint_torque()->mutable_duration()->set_seconds(sec);
        m_message.mutable_apply_joint_torque()->mutable_duration()->set_nano_seconds(nsec);

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

        /// Launch ApplyJointTorque message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::ApplyJointTorqueMsgHandler>(m_world,
                                                                                    m_commander));

        // Starts thread for simulation begin update
        m_worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&applyJointTorqueTest::onWorldUpdateStart, this));

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
 * It tests success of the apply joint torque
 * Also, it tests the client successfully gets the NO Error message
 */
TEST_F(applyJointTorqueTest, testTorqueSuccess) {
    // pause the world and torque should be zero
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    std::string modelName = "2_body";
    std::string jointName = "follower_joint";
    auto model = m_world->ModelByName(modelName);
    auto joint = model->GetJoint(jointName);
    uint32_t axis = 0;
    ASSERT_EQ(0, joint->GetForce(axis));

    // apply torque for 1 second and check for the next iteration
    double effort = 1;
    uint64_t applyTime = 1;
    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyJointTorque(modelName, jointName, axis, effort, applyTime, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1000);
    EXPECT_DOUBLE_EQ(joint->GetVelocity(axis), effort);

    // check for next step and velocity should be kept the same
    for (size_t idx = 0; idx < 100; ++idx) {
        m_world->Step(1);
        EXPECT_DOUBLE_EQ(joint->GetVelocity(axis), effort);
    }
}

/*
 * It tests success of the apply joint torque with overlapping time frames
 */
TEST_F(applyJointTorqueTest, testTorqueOverlap) {
    // pause the world and torque should be zero
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    std::string modelName = "2_body";
    std::string jointName = "follower_joint";
    auto model = m_world->ModelByName(modelName);
    auto joint = model->GetJoint(jointName);
    uint32_t axis = 0;
    ASSERT_EQ(joint->GetForce(axis), 0);

    // apply torque for 1 second twice and check for the next 2 iteration
    double effort = 1;
    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyJointTorque(modelName, jointName, axis, effort, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    effort = 0.5;
    reply = clientApplyJointTorque(modelName, jointName, axis, effort, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1000);
    EXPECT_DOUBLE_EQ(joint->GetVelocity(axis), effort);

    // check for next step and velocity should be kept at original value
    m_world->Step(100);
    EXPECT_DOUBLE_EQ(joint->GetVelocity(axis), effort);
}

/*
 * It tests success of the apply joint torque on two axis at the same time
 */
TEST_F(applyJointTorqueTest, testTorqueTwoAxis) {
    // pause the world and torque should be zero
    m_world->Run();
    m_world->SetPaused(true);
    m_world->ResetTime();

    std::string modelName = "3_body";
    std::string jointName = "follower_joint";
    auto model = m_world->ModelByName(modelName);
    auto joint = model->GetJoint(jointName);
    uint32_t axisA = 0;
    uint32_t axisB = 1;
    ASSERT_EQ(joint->GetForce(axisA), 0);
    ASSERT_EQ(joint->GetForce(axisB), 0);

    // apply torque for 1 second on axisA and axisB and check for the next 2 iteration
    double effort = 1;
    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyJointTorque(modelName, jointName, axisA, effort, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    reply = clientApplyJointTorque(modelName, jointName, axisB, effort, 1, 0);
    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_NONE); // 0: NO Error

    // step simulator for 1 second and check for terminal velocity
    m_world->Step(1000);
    EXPECT_NEAR(joint->GetVelocity(axisA), effort, 1e-5);
    EXPECT_NEAR(joint->GetVelocity(axisB), effort, 1e-5);

    // check for next step and velocity should be kept at original value
    double omegaA = joint->GetVelocity(axisA);
    double omegaB = joint->GetVelocity(axisB);
    m_world->Step(100);
    EXPECT_DOUBLE_EQ(joint->GetVelocity(axisA), omegaA);
    EXPECT_DOUBLE_EQ(joint->GetVelocity(axisB), omegaB);
}


/*
 * It tests reply message for invalid model name as input.
 * The client should successfully receive the MODEL_NAME_INVALID error message
 */
TEST_F(applyJointTorqueTest, testInValidModelName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyJointTorque("unit_box0", "joint", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(),
              mw::internal::robotics::gazebotransport::Packet_CoSimError::
                  Packet_CoSimError_MODEL_NAME_INVALID); // MODEL_NAME_INVALID
                                                         // error
}

/*
 * It tests reply message for invalid joint name as input.
 * The client should successfully receive the JOINT_NAME_INVALID error message
 */
TEST_F(applyJointTorqueTest, testInValidJointName) {
    mw::internal::robotics::gazebotransport::Packet reply =
        clientApplyJointTorque("unit_box", "joint0", 0, 1000.0, 0, 0);

    ASSERT_EQ(reply.status(), mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_JOINT_NAME_INVALID); // JOINT_NAME_INVALID error
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
